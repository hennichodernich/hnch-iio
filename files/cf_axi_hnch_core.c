/*
 * AXI_HNCH ADI ADC Interface Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/fmc/ad9467
 */

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "cf_axi_hnch.h"

#include "si5351_defs.h"


#define AIM_CHAN(_chan, _si, _bits, _sign)							\
	{ .type = IIO_VOLTAGE,											\
	  .indexed = 1,													\
	  .channel = _chan,												\
	  .info_mask_separate = 0,                       				\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) |  	\
	  	  	  	  	  	  	  	  BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
								  BIT(IIO_CHAN_INFO_FREQUENCY),	    \
	  .ext_info = NULL,												\
	  .scan_index = _si,											\
	  .scan_type = {												\
		.sign = _sign,												\
		.realbits = _bits,											\
		.storagebits = 16,											\
		.shift = 0,													\
	  },															\
	}


struct hnchadc_core_info {
	unsigned int version;
};

static const unsigned long dummy_available_scan_masks[] = {
	0x0F, 0x03, 0x0C, 0x05, 0
};

static const struct hnchadc_chip_info dummychip_info = {
		.name = "dummychip",
		.max_rate = 40000000UL,
		.max_testmode = 0,
		.num_channels = 4,
		.scan_masks = dummy_available_scan_masks,
		.channel[0] = AIM_CHAN(0, 0, 12, 'S'),
		.channel[1] = AIM_CHAN(1, 1, 12, 'S'),
		.channel[2] = AIM_CHAN(2, 2, 12, 'S'),
		.channel[3] = AIM_CHAN(3, 3, 12, 'S'),
	};

static struct attribute *dummy_phy_attributes[] = {
	NULL,
};

static const struct attribute_group dummy_phy_attribute_group = {
	.attrs = dummy_phy_attributes,
};

static int hnchadc_chan_to_regoffset(struct iio_chan_spec const *chan)
{
	if (chan->modified)
		return chan->scan_index;

	return chan->channel;
}


static unsigned int hnchadc_num_phys_channels(struct hnchadc_state *st)
{
	struct hnchadc_converter *conv = to_converter(st->dev_i2c);
	return dummychip_info.num_channels;
}


static int hnchadc_get_physical_sampling_frequency(struct hnchadc_state *st, unsigned long *freq)
{
	struct iio_dev *synth_indio_dev = i2c_get_clientdata(to_i2c_client(st->dev_i2c));
	struct si5351_state *synth_st = iio_priv(synth_indio_dev);
	int ret = 0;

	*freq = synth_st->freq_cache[ADCCLK];
	return 0;
}

static ssize_t hnchadc_sampling_frequency_available(struct device *dev,
						   struct device_attribute *attr,
						   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hnchadc_state *st = iio_priv(indio_dev);
	unsigned long freq;
	int i, ret;


	ret = snprintf(buf, PAGE_SIZE, "[125000 1 40000000]\n");

//	mutex_lock(&indio_dev->mlock);
//	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_DEVICE_ATTR(in_voltage_sampling_frequency_available, S_IRUGO,
		       hnchadc_sampling_frequency_available,
		       NULL,
		       0);

static struct attribute *hnchadc_attributes[] = {
	&iio_dev_attr_in_voltage_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group hnchadc_dec_attribute_group = {
	.attrs = hnchadc_attributes,
};

static int hnchadc_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct hnchadc_state *st = iio_priv(indio_dev);
	struct iio_dev *vga_indio_dev = spi_get_drvdata(to_spi_device(st->dev_spi));
	int ret, sign;
	unsigned tmp, phase = 0, channel;
	unsigned long long llval;
	unsigned long freq = 0;
	struct iio_info *vga_info;
	struct iio_chan_spec command;

	channel = hnchadc_chan_to_regoffset(chan);

	vga_info=vga_indio_dev->info;
	if (vga_info==NULL){
		dev_err(&indio_dev->dev, "VGA IIO device has no info attached\n");
		return -EINVAL;
	}

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		*val2 = 0;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OFFSET:
		*val = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		hnchadc_get_physical_sampling_frequency(st, &freq);
		*val = freq / st->decimation_factor;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		mutex_lock(&indio_dev->mlock);
		command.channel=0;
		(vga_info->read_raw)(vga_indio_dev, &command, val, val2, IIO_CHAN_INFO_HARDWAREGAIN);
		command.channel=1;
		(vga_info->read_raw)(vga_indio_dev, &command, val, val2, IIO_CHAN_INFO_HARDWAREGAIN);
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_FREQUENCY:
		hnchadc_get_physical_sampling_frequency(st, &freq);
		freq *= st->band_select;
		*val = freq / st->decimation_factor;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int hnchadc_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct hnchadc_state *st = iio_priv(indio_dev);
	struct iio_dev *synth_indio_dev = i2c_get_clientdata(to_i2c_client(st->dev_i2c));
	struct iio_dev *vga_indio_dev = spi_get_drvdata(to_spi_device(st->dev_spi));
	unsigned long long llval;
	struct iio_chan_spec *synth_channels;
	struct iio_info *vga_info;
	struct iio_chan_spec command;
	struct iio_chan_spec_ext_info *synth_ddrclk_ext_info, *synth_adcclk_ext_info;
	int len, ret, synth_freq;
	unsigned long tmp;
	unsigned char freq_string[100];
	unsigned int regval, dec_bit=0, i;

	synth_channels=synth_indio_dev->channels;
	if (synth_channels==NULL){
		dev_err(&indio_dev->dev, "Synthesizer IIO device has no channels\n");
		return -EINVAL;
	}

	synth_ddrclk_ext_info=synth_channels[DDRCLK].ext_info;
	if (synth_ddrclk_ext_info==NULL){
		dev_err(&indio_dev->dev, "Synthesizer channel %d has no info attached\n", DDRCLK);
		return -EINVAL;
	}
	if (synth_ddrclk_ext_info->write==NULL){
		dev_err(&indio_dev->dev, "Synthesizer channel %d has no write function attached\n", DDRCLK);
		return -EINVAL;
	}

	synth_adcclk_ext_info=synth_channels[ADCCLK].ext_info;
	if (synth_adcclk_ext_info==NULL){
		dev_err(&indio_dev->dev, "Synthesizer channel %d has no info attached\n", ADCCLK);
		return -EINVAL;
	}
	if (synth_adcclk_ext_info->write==NULL){
		dev_err(&indio_dev->dev, "Synthesizer channel %d has no write function attached\n", ADCCLK);
		return -EINVAL;
	}

	vga_info=vga_indio_dev->info;
	if (vga_info==NULL){
		dev_err(&indio_dev->dev, "VGA IIO device has no info attached\n");
		return -EINVAL;
	}

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		dev_dbg(&indio_dev->dev, "cf_axi_hnch: sampling frequency %d requested\n", val);

		if (val==0){
			return 0;
		}

		if ((val<125000) || (val>40000000)){
			dev_err(&indio_dev->dev, "Sampling frequency out of range\n");
			return -EINVAL;
		}

		if (val<=5000000){
			st->decimation_factor = 8;
			st->decimation_enable = 1;
		}
		else{
			st->decimation_factor = 1;
			st->decimation_enable = 0;
		}

		synth_freq = st->decimation_factor * val;
		st->adc_clk = synth_freq;

		if (synth_freq==4000000){
			st->led_color = 1;
		}else if (synth_freq==8000000){
			st->led_color = 2;
		}else if (synth_freq==10000000){
			st->led_color = 4;
		}else if (synth_freq==20000000){
			st->led_color = 6;
		}else if (synth_freq==24000000){
			st->led_color = 3;
		}else if (synth_freq==40000000){
			st->led_color = 5;
		}else{
			st->led_color = 0;
		}
		hnchadc_write(st, 0, ( (st->led_color         & 0x07) << LED_OFFSET) \
							| ((st->decimation_enable & 0x01) << DECIMATE_OFFSET) \
							| ((st->band_select       & 0x07) << BAND_SELECT_OFFSET) \
							| ((st->stream_select     & 0x03) << STREAM_SELECT_OFFSET) );

		mutex_lock(&indio_dev->mlock);

		len=snprintf(freq_string,100,"%ld",synth_freq);
		command.channel=DDRCLK;
		dev_dbg(&indio_dev->dev, "cf_axi_hnch: writing %s for channel %d on synthesizer IIO device\n", freq_string, command.channel);
		ret=(synth_ddrclk_ext_info->write)(synth_indio_dev, SI5351_FREQ, &command, freq_string, len);
		command.channel=ADCCLK;
		dev_dbg(&indio_dev->dev, "cf_axi_hnch: writing %s for channel %d on synthesizer IIO device\n", freq_string, command.channel);
		ret=(synth_adcclk_ext_info->write)(synth_indio_dev, SI5351_FREQ, &command, freq_string, len);

		command.channel=DDRCLK;
		(synth_ddrclk_ext_info->write)(synth_indio_dev, SI5351_PHASE, &command, "0", 1);
		command.channel=ADCCLK;
		(synth_adcclk_ext_info->write)(synth_indio_dev, SI5351_PHASE, &command, "90", 2);

		mutex_unlock(&indio_dev->mlock);

		if (ret>0){
			return 0;
		}
		else{
			return ret;
		}
	case IIO_CHAN_INFO_HARDWAREGAIN:
		mutex_lock(&indio_dev->mlock);

		command.channel=0;
		(vga_info->write_raw)(vga_indio_dev, &command, val, val2, IIO_CHAN_INFO_HARDWAREGAIN);
		command.channel=1;
		(vga_info->write_raw)(vga_indio_dev, &command, val, val2, IIO_CHAN_INFO_HARDWAREGAIN);

		mutex_unlock(&indio_dev->mlock);
		return 0;
	case IIO_CHAN_INFO_FREQUENCY:
		hnchadc_get_physical_sampling_frequency(st, &tmp);
		if (tmp==0)
			return -EINVAL;
		if (val>=tmp)
			return -EINVAL;

		st->band_select = val * 8 / tmp;
		dev_dbg(&indio_dev->dev, "cf_axi_hnch: selecting band no. %d around %d Hz for decimation\n", st->band_select, tmp*st->band_select/8);

		hnchadc_write(st, 0, ( (st->led_color         & 0x07) << LED_OFFSET) \
							| ((st->decimation_enable & 0x01) << DECIMATE_OFFSET) \
							| ((st->band_select       & 0x07) << BAND_SELECT_OFFSET) \
							| ((st->stream_select     & 0x03) << STREAM_SELECT_OFFSET) );
		return 0;
	default:
		return -EINVAL;
	}
}
#if 0
static int hnchadc_read_event_value(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int *val,
	int *val2)
{
	return -ENOSYS;
}

static int hnchadc_write_event_value(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int val,
	int val2)
{
	return -ENOSYS;
}

static int hnchadc_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	return -ENOSYS;
}

static int hnchadc_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int state)
{
	return -ENOSYS;
}
#endif

static int hnchadc_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *scan_mask)
{
	struct hnchadc_state *st = iio_priv(indio_dev);

	printk(KERN_INFO "cf_axi_hnch: hnchadc_update_scan_mode called with scan mask %x\n",*scan_mask);

	if (*scan_mask==0x03)
	{
		st->stream_select=0;
	}
	else if (*scan_mask==0x0C)
    {
		st->stream_select=1;
	}
	else if (*scan_mask==0x05)
	{
		st->stream_select=2;
	}
	else if (*scan_mask==0x0F)
	{
		st->stream_select=3;
	}
	else
	{
		return -1;
	}
	hnchadc_write(st, 0, ( (st->led_color         & 0x07) << LED_OFFSET) \
								| ((st->decimation_enable & 0x01) << DECIMATE_OFFSET) \
								| ((st->band_select       & 0x07) << BAND_SELECT_OFFSET) \
								| ((st->stream_select     & 0x03) << STREAM_SELECT_OFFSET) );
	return 0;
}


static int hnchadc_channel_setup(struct iio_dev *indio_dev,
				const struct iio_chan_spec *adc_channels,
				unsigned adc_chan_num)
{
	struct hnchadc_state *st = iio_priv(indio_dev);
	unsigned i, cnt, usr_ctrl;
	printk(KERN_INFO "cf_axi_hnch: entered hnchadc_channel_setup, indio_dev pointer: %08x, adc_channels pointer: %08x, st pointer: %08x\n",indio_dev, adc_channels, st);

	for (i = 0, cnt = 0; i < adc_chan_num; i++)
		st->channels[cnt++] = adc_channels[i];

	indio_dev->channels = st->channels;
	indio_dev->num_channels = cnt;
	indio_dev->masklength = cnt;

	return 0;
}

static const struct iio_info hnchadc_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &hnchadc_read_raw,
	.write_raw = &hnchadc_write_raw,
	.update_scan_mode = &hnchadc_update_scan_mode,
};

static const struct hnchadc_core_info dummycore_info = {
	.version = 0,
};
static const struct of_device_id hnchadc_of_match[] = {
	{ .compatible = "xlnx,hnch-ctrl-1.0", .data = &dummycore_info },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, hnchadc_of_match);

static int hnchadc_attach_i2c_client(struct device *dev, void *data)
{
	struct hnchadc_i2cdev *i2cdev = data;
	int ret = 0;

	device_lock(dev);
	if ((i2cdev->of_i2c == dev->of_node) && dev->driver) {
		i2cdev->dev_i2c = dev;
		ret = 1;
	}
	device_unlock(dev);

	return ret;
}

static int hnchadc_attach_spi_client(struct device *dev, void *data)
{
	struct hnchadc_spidev *hnchadc_spidev = data;
	int ret = 0;

	device_lock(dev);
	if ((hnchadc_spidev->of_nspi == dev->of_node) && dev->driver) {
		hnchadc_spidev->dev_spi = dev;
		ret = 1;
	}
	device_unlock(dev);

	return ret;
}



/**
 * hnchadc_of_probe - probe method for the AIM device.
 * @of_dev:	pointer to OF device structure
 * @match:	pointer to the structure used for matching a device
 *
 * This function probes the AIM device in the device tree.
 * It initializes the driver data structure and the hardware.
 * It returns 0, if the driver is bound to the AIM device, or a negative
 * value if there is an error.
 */
static int hnchadc_probe(struct platform_device *pdev)
{
	const struct hnchadc_core_info *info;
	const struct of_device_id *id;
	struct iio_dev *indio_dev, *slave_indio_devs[2];
	struct hnchadc_state *st;
	struct resource *mem;
	struct hnchadc_i2cdev i2cdev;
	struct hnchadc_spidev spidev;
	int ret;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
		 pdev->dev.of_node->name);

	id = of_match_node(hnchadc_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;
	printk(KERN_INFO "cf_axi_hnch: found matching node in device tree\n");
	info = id->data;

	i2cdev.of_i2c = of_parse_phandle(pdev->dev.of_node,
						 "i2cbus-connected", 0);
	if (!i2cdev.of_i2c) {
		dev_err(&pdev->dev, "could not find I2C node\n");
		return -ENODEV;
	}

	ret = bus_for_each_dev(&i2c_bus_type, NULL, &i2cdev,
			       hnchadc_attach_i2c_client);
	if (ret == 0)
		return -EPROBE_DEFER;

	if (!try_module_get(i2cdev.dev_i2c->driver->owner))
		return -ENODEV;

	printk(KERN_INFO "cf_axi_hnch: found correct I2C device, allocating\n");

	get_device(i2cdev.dev_i2c);


	spidev.of_nspi = of_parse_phandle(pdev->dev.of_node,
							 "spibus-connected", 0);
	if (!spidev.of_nspi) {
		dev_err(&pdev->dev, "could not find SPI node\n");
		return -ENODEV;
	}

	ret = bus_for_each_dev(&spi_bus_type, NULL, &spidev,
				       hnchadc_attach_spi_client);
	if (ret == 0)
		return -EPROBE_DEFER;

	if (!try_module_get(spidev.dev_spi->driver->owner))
		return -ENODEV;

	printk(KERN_INFO "cf_axi_hnch: found correct SPI device, allocating\n");

	get_device(spidev.dev_spi);



	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto err_put_converter;
	}
	dev_dbg(&pdev->dev, "alloc'ed iio device\n");
	st = iio_priv(indio_dev);
	dev_dbg(&pdev->dev, "got state\n");

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs_size = resource_size(mem);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	hnchadc_write(st, 0, 0x3F);	//RGB LED White
	msleep_interruptible(500);
	hnchadc_write(st, 0, 0);	//RGB LED Off

	st->led_color = 0;
	st->decimation_factor = 1;
	st->decimation_enable = 0;
	st->band_select = 0;
	st->stream_select = 3;

	st->dev_i2c = i2cdev.dev_i2c;
	st->dev_spi = spidev.dev_spi;

	platform_set_drvdata(pdev, indio_dev);
	dev_dbg(&pdev->dev, "set drvdata\n");

	slave_indio_devs[0] = i2c_get_clientdata(to_i2c_client(st->dev_i2c));

	if (IS_ERR(slave_indio_devs[0])) {
		dev_err(&pdev->dev, "Failed to get synthesizer IIO device: %d\n",
				(int)PTR_ERR(slave_indio_devs[0]));
		return PTR_ERR(slave_indio_devs[0]);
	}
	printk(KERN_INFO "cf_axi_hnch: got synthesizer IIO device %s\n",slave_indio_devs[0]->name);

	slave_indio_devs[1] = spi_get_drvdata(to_spi_device(st->dev_spi));

	if (IS_ERR(slave_indio_devs[1])) {
		dev_err(&pdev->dev, "Failed to get VGA IIO device: %d\n",
				(int)PTR_ERR(slave_indio_devs[1]));
		return PTR_ERR(slave_indio_devs[1]);
	}
	printk(KERN_INFO "cf_axi_hnch: got VGA IIO device %s\n",slave_indio_devs[1]->name);

	iio_device_set_drvdata(indio_dev, slave_indio_devs);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = dummychip_info.scan_masks;
	printk(KERN_INFO "cf_axi_hnch: filled indio_dev\n");

	hnchadc_channel_setup(indio_dev, dummychip_info.channel, dummychip_info.num_channels);
	printk(KERN_INFO "cf_axi_hnch: set up channel\n");

	st->iio_info = hnchadc_info;
	st->iio_info.attrs = &dummy_phy_attribute_group;
	printk(KERN_INFO "cf_axi_hnch: set phy attrs\n");
	indio_dev->info = &st->iio_info;

	if (of_find_property(pdev->dev.of_node, "dmas", NULL)) {
		ret = hnchadc_configure_ring_stream(indio_dev, NULL);
		if (ret < 0)
			goto err_put_converter;
	}
	printk(KERN_INFO "cf_axi_hnch: configured ring buffer\n");

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_unconfigure_ring;
	printk(KERN_INFO "cf_axi_hnch: registered IIO device\n");
	dev_info(&pdev->dev, "simple driver probed ADC %s\n",dummychip_info.name);
	printk(KERN_INFO "cf_axi_hnch: simple driver probed ADC %s\n",dummychip_info.name);

	return 0;
err_unconfigure_ring:
	hnchadc_unconfigure_ring_stream(indio_dev);

err_put_converter:
	put_device(i2cdev.dev_i2c);
	module_put(i2cdev.dev_i2c->driver->owner);

	return ret;
}

/**
 * hnchadc_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int hnchadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct hnchadc_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	put_device(st->dev_i2c);
	module_put(st->dev_i2c->driver->owner);

	return 0;
}

static struct platform_driver hnchadc_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = hnchadc_of_match,
	},
	.probe		= hnchadc_probe,
	.remove		= hnchadc_remove,
};

module_platform_driver(hnchadc_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>, Henning Paul <hnch@gmx.net>");
MODULE_DESCRIPTION("hnch DMA IIO driver");
MODULE_LICENSE("GPL v2");

/*
Hnch ADC driver
 */

#ifndef CF_AXI_HNCH_H_
#define CF_AXI_HNCH_H_

#include <linux/i2c.h>


#define DDRCLK 2
#define ADCCLK 1

#define LED_OFFSET 0
#define DECIMATE_OFFSET 8
#define BAND_SELECT_OFFSET 9
#define STREAM_SELECT_OFFSET 12

#define AXIADC_MAX_CHANNEL		4

struct hnchadc_chip_info {
	char				*name;
	unsigned			num_channels;
//	unsigned 		num_shadow_slave_channels;
	const unsigned long 	*scan_masks;
	const int			(*scale_table)[2];
//	int				num_scales;
	int				max_testmode;
	unsigned long			max_rate;
	struct iio_chan_spec		channel[AXIADC_MAX_CHANNEL];
};

struct hnchadc_state {
	struct device 			*dev_i2c;
	struct device 			*dev_spi;
	struct iio_info			iio_info;
	size_t				regs_size;
	void __iomem			*regs;
	unsigned			decimation_factor;
	unsigned 			decimation_enable;
	unsigned	band_select;
	unsigned	stream_select;
	unsigned long long		adc_clk;
	unsigned	led_color;

	struct iio_hw_consumer		*frontend;

	struct iio_chan_spec		channels[AXIADC_MAX_CHANNEL];
};

struct hnchadc_converter {
	struct i2c_device 	*i2c;
	unsigned long 		adc_clk;
	const struct hnchadc_chip_info	*chip_info;
	bool			sample_rate_read_only;

	struct iio_chan_spec const	*channels;
	int				num_channels;
	const struct attribute_group	*attrs;
	struct iio_dev 	*indio_dev;
	int (*read_raw)(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask);

	int (*write_raw)(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int val,
			 int val2,
			 long mask);

	int (*read_event_value)(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			enum iio_event_type type,
			enum iio_event_direction dir,
			enum iio_event_info info,
			int *val,
			int *val2);

	int (*write_event_value)(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			enum iio_event_type type,
			enum iio_event_direction dir,
			enum iio_event_info info,
			int val,
			int val2);

	int (*read_event_config)(struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan,
			enum iio_event_type type,
			enum iio_event_direction dir);

	int (*write_event_config)(struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan,
			enum iio_event_type type,
			enum iio_event_direction dir,
			int state);

	int (*post_setup)(struct iio_dev *indio_dev);
	int (*post_iio_register)(struct iio_dev *indio_dev);
	int (*set_pnsel)(struct iio_dev *indio_dev, unsigned chan,
			enum adc_pn_sel sel);
};



static inline struct hnchadc_converter *to_converter(struct device *dev)
{
	struct hnchadc_converter *conv = i2c_get_clientdata(to_i2c_client(dev));

	if (conv)
		return conv;

	return ERR_PTR(-ENODEV);
};

struct hnchadc_spidev {
	struct device_node *of_nspi;
	struct device *dev_spi;
};

struct hnchadc_i2cdev {
	struct device_node *of_i2c;
	struct device *dev_i2c;
};

/*
 * IO accessors
 */

static inline void hnchadc_write(struct hnchadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int hnchadc_read(struct hnchadc_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

int hnchadc_configure_ring_stream(struct iio_dev *indio_dev,
	const char *dma_name);
void hnchadc_unconfigure_ring_stream(struct iio_dev *indio_dev);

#endif /* CF_AXI_HNCH_H_ */

/*
 * Copyright (c) 2026, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/rtio/work.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor_clock.h>

#include "bno055.h"

LOG_MODULE_REGISTER(BNO055, CONFIG_SENSOR_LOG_LEVEL);

static int bno055_rtio_sample_fetch(const struct device *dev, int16_t readings[3])
{
	uint8_t status;
	const struct bno055_dev_cfg *cfg = dev->config;
	struct bno055_data *data = dev->data;
	int err;
	struct bno055_vector3_data acc;

	switch (data->mode) {
	case BNO055_MODE_ACC_ONLY:
		err = bno055_vector3_fetch(dev, BNO055_REGISTER_ACC_DATA, &acc);
		if (err < 0) {
			return err;
		}
		break;
	}

	readings[0] = acc.x;
	readings[1] = acc.y;
	readings[2] = acc.z;

	return 0;
}

void bno055_submit_one_shot_sync(struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	const struct device *dev = cfg->sensor;
	const struct sensor_chan_spec *const channels = cfg->channels;
	const size_t num_channels = cfg->count;
	uint32_t min_buf_len = sizeof(struct bno055_encoded_data);
	int rc;
	uint8_t *buf;
	uint32_t buf_len;
	struct bno055_encoded_data *edata;

	/* Get the buffer for the frame, it may be allocated dynamically by the rtio context */
	rc = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, &buf_len);
	if (rc != 0) {
		LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	edata = (struct bno055_encoded_data *)buf;

	rc = bno055_encode(dev, channels, num_channels, buf);
	if (rc != 0) {
		LOG_ERR("Failed to encode sensor data");
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	rc = bno055_rtio_sample_fetch(dev, edata->readings);
	/* Check that the fetch succeeded */
	if (rc != 0) {
		LOG_ERR("Failed to fetch samples");
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	rtio_iodev_sqe_ok(iodev_sqe, 0);
}

void bno055_submit_one_shot(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
    struct rtio_work_req *req = rtio_work_req_alloc();

	if (req == NULL) {
		LOG_ERR("RTIO work item allocation failed. Consider to increase "
			"CONFIG_RTIO_WORKQ_POOL_ITEMS.");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	rtio_work_req_submit(req, iodev_sqe, bno055_submit_one_shot_sync);

}

void sensor_bno055_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
    const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;

    if (!cfg->is_streaming) {
		bno055_submit_one_shot(dev, iodev_sqe);
	} else if (IS_ENABLED(CONFIG_BNO055_STREAM)) {
		bno055_submit_stream(dev, iodev_sqe);
	} else {
		rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
	}
}
/*
 * msm_debugfs.h
 *
 *  Created on: Jan 13, 2015
 *      Author: charles
 */

#ifndef DRIVER_SENSOR_DEBUGFS_MSM_DEBUGFS_H_
#define DRIVER_SENSOR_DEBUGFS_MSM_DEBUGFS_H_
#include "../msm_sensor.h"
#include "../actuator/msm_actuator.h"
#include "../cci/msm_cci.h"
#include "../ois/msm_ois.h"

#define	REAR_OTP_THERMAL_FILE	"driver/rear_temperature"
#define	REAR_OTP_PROC_FILE		"driver/rear_otp"
#define	FRONT_OTP_PROC_FILE		"driver/front_otp"
#define	OIS_RW_PROC_FILE		"driver/ois_rw"
#define	OIS_READ_TIMES_PROC_FILE		"driver/ois_read_times"
#define	IMX318_RW_PROC_FILE		"driver/imx318_rw"
#define	VCM_Z1_Z2_PROC_FILE		"driver/vcm_Z1_Z2"
#define	VCM_MIN_MAX_PROC_FILE		"driver/vcm_min_max"

struct otp_struct {
	u8 af_inf[2];
	u8 af_30cm[2];
	u8 af_5cm[2];
	u8 start_current[2];
	u8 module_id;
	u8 vendor_id;
	u8 dc[4];
	u8 sn[4];
	u8 pn[4];
	u8 code[2];
	u8 zero[6];
	u8 cks[2];
};

struct debugfs {
	u8 status;
	u8 exposure_return0;
	u16 sensor_id;
};
extern int msm_debugfs_init(struct msm_sensor_ctrl_t *s_ctrl, struct msm_camera_sensor_slave_info *slave_info);
extern void msm_debugfs_set_status(unsigned int camera_id, unsigned int status);
extern int msm_read_otp(struct msm_sensor_ctrl_t *s_ctrl, struct msm_camera_sensor_slave_info *slave_info);
extern void msm_set_actuator_ctrl(struct msm_actuator_ctrl_t *s_ctrl);
extern void vcm_Z1_Z2_read(uint16_t *actuator_data_Z1, uint16_t *actuator_data_Z2);
extern void msm_set_ois_ctrl(struct msm_ois_ctrl_t *o_ctrl);
extern void get_eeprom_OTP(struct msm_eeprom_memory_block_t *block);
extern void imx318_power_state(int power_on);
extern int i2c_write_thu_imx318(uint8_t *value, int num);
extern int i2c_read_thu_imx318(uint8_t *value, int num, uint8_t *read_data, int read_num);

#endif /* DRIVER_SENSOR_DEBUGFS_MSM_DEBUGFS_H_ */

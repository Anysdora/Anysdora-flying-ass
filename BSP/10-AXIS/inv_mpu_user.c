#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu_user.h"
#include "stdint.h"
#include <math.h>
#include "include.h"

#define MPL_LOGI(...)     do {} while (0)
#define MPL_LOGE(...)     do {} while (0)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
 /*.orientation = */{ 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
 /*.orientation = */{ 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
 /*.orientation = */{-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
 /*.orientation = */{-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif

static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;		// error
    return b;
}

/** Converts an orientation matrix made up of 0,+1,and -1 to a scalar representation.
* @param[in] mtx Orientation matrix to convert to a scalar.
* @return Description of orientation matrix. The lowest 2 bits (0 and 1) represent the column the one is on for the
* first row, with the bit number 2 being the sign. The next 2 bits (3 and 4) represent
* the column the one is on for the second row with bit number 5 being the sign.
* The next 2 bits (6 and 7) represent the column the one is on for the third row with
* bit number 8 being the sign. In binary the identity matrix would therefor be:
* 010_001_000 or 0x88 in hex.
*/
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{

    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

void run_self_test(void)
{
    int result;
//  char test_packet[4] = {0};
    long gyro[3], accel[3];
	//
    result = mpu_run_self_test(gyro, accel);
  if (result == 0x7) 
//		if (result == 0x3) 
		{
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
}

void dmp_set_bias(void)
{
	long gyro[3]={0xfffe6cdb,0x00172b8c,0xfff914af};
	long accel[3]={0xffd68000,0x02530000,0xff688000};
	dmp_set_accel_bias(accel);
	dmp_set_gyro_bias(gyro);
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}

int mpu_dmp_init(void)
{
	int result;
    unsigned long timestamp;
    struct int_param_s int_param;
	
	result = mpu_init(&int_param);
	if (result) 
	{
		return result;
	}

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
#ifdef COMPASS_ENABLED
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
    /* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

  /* Compass reads are handled by scheduler. */
  get_ms_gansha(&timestamp);

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
//    dmp_register_tap_cb(tap_cb);
//    dmp_register_android_orient_cb(android_orient_cb);
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
     */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
		run_self_test();
//		dmp_set_bias();
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;

//		估计是后续添加的代码
//		mpu_get_accel_fsr(&sen.accel_fsr);
//		mpu_get_gyro_fsr(&sen.gyro_fsr);

	return result;
}

void inv_mpu_read(struct _angle *angle, struct _sensor *sensor)
{
	unsigned char  new_temp = 0;
    unsigned long timestamp;

#ifdef COMPASS_ENABLED
    unsigned char new_compass = 0;
#endif
	
    unsigned long sensor_timestamp;

    get_ms_gansha(&timestamp);

#ifdef COMPASS_ENABLED
        /* We're not using a data ready interrupt for the compass, so we'll
         * make our compass reads timer-based instead.
         */
        if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode &&
            hal.new_gyro && (hal.sensors & COMPASS_ON)) {
            hal.next_compass_ms = timestamp + COMPASS_READ_MS;
            new_compass = 1;
        }
#endif
        /* Temperature data doesn't need to be read with every gyro sample.
         * Let's make them timer-based like the compass reads.
         */
        if (timestamp > hal.next_temp_ms) 
		{
            hal.next_temp_ms = timestamp + TEMP_READ_MS;
            new_temp = 1;
        }

//    if (!hal.sensors || !hal.new_gyro) 
//	{
//        return;
//    }    

//	if (hal.new_gyro && hal.dmp_on) 
//	{
		short gyro[3], accel_short[3], sensors;
		long quat[4];
		unsigned char more;
				
				/* This function gets new data from the FIFO when the DMP is in
				 * use. The FIFO can contain any combination of gyro, accel,
				 * quaternion, and gesture data. The sensors parameter tells the
				 * caller which data fields were actually populated with new data.
				 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
				 * the FIFO isn't being filled with accel data.
				 * The driver parses the gesture data to determine if a gesture
				 * event has occurred; on an event, the application will be notified
				 * via a callback (assuming that a callback function was properly
				 * registered). The more parameter is non-zero if there are
				 * leftover packets in the FIFO.
				 */
		if(!dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more))
		{

	//		if (!more)
	//			hal.new_gyro = 0;
			if(!more)
			{
//				mpu_reset_fifo();
			}
			
			if (sensors & INV_XYZ_GYRO)
			{
						sensor_gyro_calculation(gyro, &sensor->gyro);
				
	//					inv_build_gyro(gyro,sensor_timestamp);
	//					if (new_temp) 
	//					{
	//							new_temp = 0;
	//							/* Temperature only used for gyro temp comp. */
	//							mpu_get_temperature(&sen.temp_raw, &sensor_timestamp);
	//							sen.temp_time=sensor_timestamp;
	//					}
			}
			if (sensors & INV_XYZ_ACCEL) 
			{
						sensor_accel_calculation(accel_short, &sensor->accel);

	//					inv_build_accel(accel_short, sensor_timestamp);
			}
			if (sensors & INV_WXYZ_QUAT) 
			{

				angle_calculation(quat, angle);

	//					inv_build_quat(quat, sensor_timestamp);
			}
		}
//	}
#ifdef COMPASS_ENABLED
		if (new_compass) {
				short compass_short[3];
				new_compass = 0;
				/* For any MPU device with an AKM on the auxiliary I2C bus, the raw
				 * magnetometer registers are copied to special gyro registers.
				 */
				if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
						inv_build_compass(compass_short, sensor_timestamp);
				}
		}
#endif
}


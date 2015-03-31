/*

Adapted by Corentin Smith from https://github.com/kriswiner/GY-80 by Kris Winer

*/

#ifndef GY80_h
#define GY80_h

#include "Config.h"
#include "Utils.h"
#include "Kalman.h"
#include "I2CWrapper.h"

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)     // gyroscope measurement error in rads/s (shown as 40 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

class GY80 {
public:
    GY80() {
    };

    void setup() {
        selfTest();

        calibrateAccel(); // Calibrate accelerometers, load biases in bias registers
        initAccel(); // Initialize and configure accelerometer
        initGyro(); // Initialize and configure gyroscope
        initMagneto(); // Initialize and configure magnetometer

        delay(10);

        int i;
        zeroYaw = 0;
        zeroPitch = 0;
        zeroRoll = 0;
        for (i = 0; i < 40; ++i)
        {
            computeAngles();
            delay(10);
        }
        for (i = 0; i < 40; ++i)
        {
            computeAngles();
            delay(10);
            zeroYaw += yaw;
            zeroRoll += roll;
            zeroPitch += pitch;
        }

        zeroYaw /= i;
        zeroPitch /= i;
        zeroRoll /= i;
    };

    bool selfTest() {
        uint8_t c = i2cReadOne(ACCEL_ADDRESS, ACCEL_WHO_AM_I);
        uint8_t d = i2cReadOne(GYRO_ADDRESS, GYRO_WHO_AM_I);
        uint8_t e = i2cReadOne(MAGNETO_ADDRESS, MAGNETO_IDA);
        uint8_t f = i2cReadOne(MAGNETO_ADDRESS, MAGNETO_IDB);
        uint8_t g = i2cReadOne(MAGNETO_ADDRESS, MAGNETO_IDC);

        if (c == 0xE5) {
            DEBUG("ADXL345  is online...");
        }
        if (d == 0xD3) {
            DEBUG("L3G4200D is online...");
        }
        if (e == 0x48 && f == 0x34 && g == 0x33) {
            DEBUG("HMC5883L is online...");
        }
        if (!(c == 0xE5 && d == 0xD3 && e == 0x48 && f == 0x34 && g == 0x33)) {
            while (1) {
                DEBUG("ERROR");
                DEBUG(c);
                DEBUG(d);
                DEBUG(e);
                DEBUG(f);
                DEBUG(g);
            }
        }
    }

    void getSensorValue() {
        computeAngles();

        yaw -= zeroYaw;
        pitch -= zeroPitch;
        roll -= zeroRoll;

        Serial.println("Yaw   " + yaw);
        Serial.println("Pitch " + pitch);
        Serial.println("Roll  " + roll);
    };

    void computeAngles()
    {
        // If intPin goes high or data ready status is TRUE, all data registers have new data
        if (i2cReadOne(ACCEL_ADDRESS, ACCEL_INT_SOURCE) & 0x80) {  // When data is ready
            readAccelData(accelCount);  // Read the x/y/z adc values
            getAres();

            // Now we'll calculate the accleration value into actual g's
            ax = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
            ay = (float)accelCount[1]*aRes;
            az = (float)accelCount[2]*aRes;
        }

        if (i2cReadOne(GYRO_ADDRESS, GYRO_STATUS_REG) & 0x08) {
            readGyroData(gyroCount);  // Read the x/y/z adc values
            getGres();

            // Calculate the gyro value into actual degrees per second
            gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
            gy = (float)gyroCount[1]*gRes;
            gz = (float)gyroCount[2]*gRes;
        }

        if(i2cReadOne(MAGNETO_ADDRESS, MAGNETO_STATUS) & 0x01) { // If data ready bit set, then read magnetometer data
            readMagData(magCount);  // Read the x/y/z adc values
            mRes = 0.73; // Conversion to milliGauss, 0.73 mG/LSB in hihgest resolution mode
            // So far, magnetometer bias is calculated and subtracted here manually, should construct an algorithm to do it automatically
            // like the gyro and accelerometer biases
            magbias[0] =  -30.0f;  // User environmental x-axis correction in milliGauss
            magbias[1] =  +85.0f;  // User environmental y-axis correction in milliGauss
            magbias[2] =  -78.0f;  // User environmental z-axis correction in milliGauss

            // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental corrections
            mx = (float)magCount[0]*mRes - magbias[0];  // get actual magnetometer value, this depends on scale being set
            my = (float)magCount[1]*mRes - magbias[1];
            mz = (float)magCount[2]*mRes - magbias[2];
        }


        now = micros();
        deltat = ((now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
        lastUpdate = now;

        // In the Gy-80, all three orientation sensors' x-, y-, and z-axes are aligned;
        // the magnetometer z-axis (+ up) is parallel to z-axis (+ up) of accelerometer and gyro!
        // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
        // For the GY-80, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
        // in the LSM9DS0 sensor. We negate the z-axis magnetic field to conform to AHRS convention of magnetic z-axis down.
        // This rotation can be modified to allow any convenient orientation convention.
        // Pass gyro rate as rad/s
        mahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);
        // You can use Magdwick and Mahony filters interchangeably
        // madgwickQuaternionUpdatema(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);


        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI;
        yaw   -= 1.31; // Declination in Paris
        roll  *= 180.0f / PI;
    }

    void calibrateAccel()
    {
        uint8_t data[6] = {0, 0, 0, 0, 0, 0};
        int abias[3] = {0, 0, 0};
        int16_t accel_bias[3] = {0, 0, 0};
        int samples, ii;

        // wake up device
        i2cWrite(ACCEL_ADDRESS, ACCEL_POWER_CTL, 0x00); // Put device in standby mode and clear sleep bit 2
        delay(10);  // Let device settle down
        i2cWrite(ACCEL_ADDRESS, ACCEL_POWER_CTL, 0x08); // Put device in normal mode
        delay(10);

        // Set accelerometer configuration; interrupt active high, left justify MSB
        i2cWrite(ACCEL_ADDRESS, ACCEL_DATA_FORMAT, 0x04 | 0x00); // Set full scale range to 2g for the bias calculation
        uint16_t  accelsensitivity = 256;  // = 256 LSB/g at 2g full scale

        // Choose ODR and bandwidth
        i2cWrite(ACCEL_ADDRESS, ACCEL_BW_RATE, 0x09); // Select normal power operation, and 100 Hz ODR and 50 Hz bandwidth
        delay(10);

        i2cWrite(ACCEL_ADDRESS, ACCEL_FIFO_CTL, 0x40 | 0x2F);    // Enable FIFO stream mode | collect 32 FIFO samples
        delay(1000);  // delay 1000 milliseconds to collect FIFO samples

        samples = i2cReadOne(ACCEL_ADDRESS, ACCEL_FIFO_STATUS);
        for(ii = 0; ii < samples ; ii++) {
            i2cRead(ACCEL_ADDRESS, ACCEL_DATAX0, 6, &data[0]);
            accel_bias[0] += (((int16_t)data[1] << 8) | data[0]) >> 6;
            accel_bias[1] += (((int16_t)data[3] << 8) | data[2]) >> 6;
            accel_bias[2] += (((int16_t)data[5] << 8) | data[4]) >> 6;
        }

        accel_bias[0] /= samples; // average the data
        accel_bias[1] /= samples;
        accel_bias[2] /= samples;

        // Remove gravity from z-axis accelerometer bias value
        if(accel_bias[2] > 0) {
            accel_bias[2] -= accelsensitivity;
        }
        else {
            accel_bias[2] += accelsensitivity;
        }

        abias[0] = (int)((-accel_bias[0]/4) & 0xFF); // offset register are 8 bit 2s-complement, so have sensitivity 1/4 of 2g full scale
        abias[1] = (int)((-accel_bias[1]/4) & 0xFF);
        abias[2] = (int)((-accel_bias[2]/4) & 0xFF);

        i2cWrite(ACCEL_ADDRESS, ACCEL_OFSX, abias[0]);
        i2cWrite(ACCEL_ADDRESS, ACCEL_OFSY, abias[1]);
        i2cWrite(ACCEL_ADDRESS, ACCEL_OFSZ, abias[2]);
    }

    void readAccelData(int16_t * destination)
    {
        // Read the six raw data registers into data array
        uint8_t rawData[6];
        i2cRead(ACCEL_ADDRESS, ACCEL_DATAX0, 6, &rawData[0]);
        destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
        destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
    }

    void readGyroData(int16_t * destination)
    {
        // Read the six raw data registers sequentially into data array
        uint8_t rawData[6];
        i2cRead(GYRO_ADDRESS, GYRO_OUT_X_L | 0x80, 6, &rawData[0]);
        destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)rawData[3] << 8) | rawData[1];
        destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
    }

    void readMagData(int16_t * destination)
    {
        // Read the six raw data registers sequentially into data array
        uint8_t rawData[6];
        i2cRead(MAGNETO_ADDRESS, MAGNETO_OUT_X_H, 6, &rawData[0]);
        destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];       // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)rawData[4] << 8) | rawData[5];
        destination[2] = ((int16_t)rawData[2] << 8) | rawData[3];
    }

    void getGres() {
        switch (Gscale)
        {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        case GFS_250DPS:
            gRes = 250.0/32768.0;
            break;
        case GFS_500DPS:
            gRes = 500.0/32768.0;
            break;
        case GFS_1000DPS:
            gRes = 1000.0/32768.0;
            break;
        case GFS_2000DPS:
            gRes = 2000.0/32768.0;
            break;
        }
    }

    void getAres() {
        switch (Ascale)
        {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        case AFS_2G:
            aRes = 2.0/(512.*64.);   // 10-bit 2s-complement
            break;
        case AFS_4G:
            aRes = 4.0/(1024.*32.);  // 11-bit 2s-complement
            break;
        case AFS_8G:
            aRes = 8.0/(2048.*16.);  // 12-bit 2s-complement
            break;
        case AFS_16G:
            aRes = 16.0/(4096.*8.);  // 13-bit 2s-complement
            break;
        }
    }

    void initAccel()
    {
        // wake up device
        i2cWrite(ACCEL_ADDRESS, ACCEL_POWER_CTL, 0x00); // Put device in standby mode and clear sleep bit 2
        delay(100);  // Let device settle down
        i2cWrite(ACCEL_ADDRESS, ACCEL_POWER_CTL, 0x08); // Put device in normal mode

        // Set accelerometer configuration; interrupt active high, left justify MSB
        i2cWrite(ACCEL_ADDRESS, ACCEL_DATA_FORMAT, 0x04 | Ascale); // Set full scale range for the accelerometer

        // Choose ODR and bandwidth
        i2cWrite(ACCEL_ADDRESS, ACCEL_BW_RATE, Arate); // Select normal power operation, and ODR and bandwidth

        i2cWrite(ACCEL_ADDRESS, ACCEL_INT_ENABLE, 0x80);  // Enable data ready interrupt
        i2cWrite(ACCEL_ADDRESS, ACCEL_INT_MAP, 0x00);     // Enable data ready interrupt on INT_1

        i2cWrite(ACCEL_ADDRESS, ACCEL_FIFO_CTL, 0x00);    // Bypass FIFO
    }

    void initGyro()
    {
        // Set gyro ODR to 100 Hz and Bandwidth to 25 Hz, enable normal mode
        i2cWrite(GYRO_ADDRESS, GYRO_CTRL_REG1, Grate << 4 | 0x0F);
        i2cWrite(GYRO_ADDRESS, GYRO_CTRL_REG3, 0x08);        // Push/pull, active high interrupt, enable data ready interrupt
        i2cWrite(GYRO_ADDRESS, GYRO_CTRL_REG4, Gscale << 4); // set gyro full scale
        i2cWrite(GYRO_ADDRESS, GYRO_CTRL_REG5, 0x00);        // Disable FIFO
    }

    void initMagneto()
    {
        // Set magnetomer ODR; default is 15 Hz
        i2cWrite(MAGNETO_ADDRESS, MAGNETO_CONFIG_A, Mrate << 2);
        i2cWrite(MAGNETO_ADDRESS, MAGNETO_CONFIG_B, 0x00);  // set gain (bits[7:5]) to maximum resolution of 0.73 mG/LSB
        i2cWrite(MAGNETO_ADDRESS, MAGNETO_MODE, 0x80 );     // enable continuous data mode
    }

    // Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
    // (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
    // which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
    // device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
    // The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
    // but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
    void madgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
    {
        float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
        float norm;
        float hx, hy, _2bx, _2bz;
        float s1, s2, s3, s4;
        float qDot1, qDot2, qDot3, qDot4;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q1mx;
        float _2q1my;
        float _2q1mz;
        float _2q2mx;
        float _4bx;
        float _4bz;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _2q4 = 2.0f * q4;
        float _2q1q3 = 2.0f * q1 * q3;
        float _2q3q4 = 2.0f * q3 * q4;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;
        float q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f/norm;
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f/norm;
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        _2q1mx = 2.0f * q1 * mx;
        _2q1my = 2.0f * q1 * my;
        _2q1mz = 2.0f * q1 * mz;
        _2q2mx = 2.0f * q2 * mx;
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        norm = 1.0f/norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * deltat;
        q2 += qDot2 * deltat;
        q3 += qDot3 * deltat;
        q4 += qDot4 * deltat;
        norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        norm = 1.0f/norm;
        q[0] = q1 * norm;
        q[1] = q2 * norm;
        q[2] = q3 * norm;
        q[3] = q4 * norm;

    }




    void mahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
    {
        float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
        float norm;
        float hx, hy, bx, bz;
        float vx, vy, vz, wx, wy, wz;
        float ex, ey, ez;
        float pa, pb, pc;

        // Auxiliary variables to avoid repeated arithmetic
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;
        float q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;        // use reciprocal for division
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
        hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
        bx = sqrt((hx * hx) + (hy * hy));
        bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

        // Estimated direction of gravity and magnetic field
        vx = 2.0f * (q2q4 - q1q3);
        vy = 2.0f * (q1q2 + q3q4);
        vz = q1q1 - q2q2 - q3q3 + q4q4;
        wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
        wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
        wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

        // Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
        if (Ki > 0.0f)
        {
            eInt[0] += ex;      // accumulate integral error
            eInt[1] += ey;
            eInt[2] += ez;
        }
        else
        {
            eInt[0] = 0.0f;     // prevent integral wind up
            eInt[1] = 0.0f;
            eInt[2] = 0.0f;
        }

        // Apply feedback terms
        gx = gx + Kp * ex + Ki * eInt[0];
        gy = gy + Kp * ey + Ki * eInt[1];
        gz = gz + Kp * ez + Ki * eInt[2];

        // Integrate rate of change of quaternion
        pa = q2;
        pb = q3;
        pc = q4;
        q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
        q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
        q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
        q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

        // Normalise quaternion
        norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        norm = 1.0f / norm;
        q[0] = q1 * norm;
        q[1] = q2 * norm;
        q[2] = q3 * norm;
        q[3] = q4 * norm;

    };

private:
    float pitch, yaw, roll;
    float zeroPitch, zeroYaw, zeroRoll;
    float deltat = 0.0f;        // integration interval for both filter schemes

    uint32_t lastUpdate = 0; // used to calculate integration interval
    uint32_t now = 0;        // used to calculate integration interval

    float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
    float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


    const uint8_t ACCEL_WHO_AM_I        = 0x00;   // Should return 0xE5
    const uint8_t ACCEL_THRESH_TAP      = 0x1D;   // Tap threshold
    const uint8_t ACCEL_OFSX            = 0x1E;   // X-axis offset
    const uint8_t ACCEL_OFSY            = 0x1F;   // Y-axis offset
    const uint8_t ACCEL_OFSZ            = 0x20;   // Z-axis offset
    const uint8_t ACCEL_DUR             = 0x21;   // Tap duration
    const uint8_t ACCEL_LATENT          = 0x22;   // Tap latency
    const uint8_t ACCEL_WINDOW          = 0x23;   // Tap window
    const uint8_t ACCEL_THRESH_ACT      = 0x24;   // Activity threshold
    const uint8_t ACCEL_THRESH_INACT    = 0x25;   // Inactivity threshold
    const uint8_t ACCEL_TIME_INACT      = 0x26;   // Inactivity time
    const uint8_t ACCEL_ACT_INACT_CTL   = 0x27;   // Axis enable control for activity/inactivity detection
    const uint8_t ACCEL_THRESH_FF       = 0x28;   // Free-fall threshold
    const uint8_t ACCEL_TIME_FF         = 0x29;   // Free-fall time
    const uint8_t ACCEL_TAP_AXES        = 0x2A;   // Axis control for single/double tap
    const uint8_t ACCEL_ACT_TAP_STATUS  = 0x2B;   // Source of single/double tap
    const uint8_t ACCEL_BW_RATE         = 0x2C;   // Data rate and power mode control
    const uint8_t ACCEL_POWER_CTL       = 0x2D;   // Power-saving features control
    const uint8_t ACCEL_INT_ENABLE      = 0x2E;   // Interrupt enable control
    const uint8_t ACCEL_INT_MAP         = 0x2F;   // Interrupt mapping control
    const uint8_t ACCEL_INT_SOURCE      = 0x30;   // Source of interrupts
    const uint8_t ACCEL_DATA_FORMAT     = 0x31;   // Data format control
    const uint8_t ACCEL_DATAX0          = 0x32;   // X-axis data 0
    const uint8_t ACCEL_DATAX1          = 0x33;   // X-axis data 1
    const uint8_t ACCEL_DATAY0          = 0x34;   // Y-axis data 0
    const uint8_t ACCEL_DATAY1          = 0x35;   // Y-axis data 1
    const uint8_t ACCEL_DATAZ0          = 0x36;   // Z-axis data 0
    const uint8_t ACCEL_DATAZ1          = 0x37;   // Z-axis data 1
    const uint8_t ACCEL_FIFO_CTL        = 0x38;   // FIFO control
    const uint8_t ACCEL_FIFO_STATUS     = 0x39;   // FIFO status
    const uint8_t ACCEL_ADDRESS         = 0x53;   // Device address when ADO = 0

    const uint8_t GYRO_WHO_AM_I         = 0x0F;   // Should return 0xD3
    const uint8_t GYRO_CTRL_REG1        = 0x20;
    const uint8_t GYRO_CTRL_REG2        = 0x21;
    const uint8_t GYRO_CTRL_REG3        = 0x22;
    const uint8_t GYRO_CTRL_REG4        = 0x23;
    const uint8_t GYRO_CTRL_REG5        = 0x24;
    const uint8_t GYRO_REFERENCE        = 0x25;
    const uint8_t GYRO_OUT_TEMP         = 0x26;
    const uint8_t GYRO_STATUS_REG       = 0x27;
    const uint8_t GYRO_OUT_X_L          = 0x28;
    const uint8_t GYRO_OUT_X_H          = 0x29;
    const uint8_t GYRO_OUT_Y_L          = 0x2A;
    const uint8_t GYRO_OUT_Y_H          = 0x2B;
    const uint8_t GYRO_OUT_Z_L          = 0x2C;
    const uint8_t GYRO_OUT_Z_H          = 0x2D;
    const uint8_t GYRO_FIFO_CTRL_REG    = 0x2E;
    const uint8_t GYRO_FIFO_SRC_REG     = 0x2F;
    const uint8_t GYRO_INT1_CFG         = 0x30;
    const uint8_t GYRO_INT1_SRC         = 0x31;
    const uint8_t GYRO_INT1_TSH_XH      = 0x32;
    const uint8_t GYRO_INT1_TSH_XL      = 0x33;
    const uint8_t GYRO_INT1_TSH_YH      = 0x34;
    const uint8_t GYRO_INT1_TSH_YL      = 0x35;
    const uint8_t GYRO_INT1_TSH_ZH      = 0x36;
    const uint8_t GYRO_INT1_TSH_ZL      = 0x37;
    const uint8_t GYRO_INT1_DURATION    = 0x38;
    const uint8_t GYRO_ADDRESS          = 0x69;  // Device address when ADO = 0

    const uint8_t MAGNETO_ADDRESS      = 0x1E;
    const uint8_t MAGNETO_CONFIG_A     = 0x00;
    const uint8_t MAGNETO_CONFIG_B     = 0x01;
    const uint8_t MAGNETO_MODE         = 0x02;
    const uint8_t MAGNETO_OUT_X_H      = 0x03;
    const uint8_t MAGNETO_OUT_X_L      = 0x04;
    const uint8_t MAGNETO_OUT_Z_H      = 0x05;
    const uint8_t MAGNETO_OUT_Z_L      = 0x06;
    const uint8_t MAGNETO_OUT_Y_H      = 0x07;
    const uint8_t MAGNETO_OUT_Y_L      = 0x08;
    const uint8_t MAGNETO_STATUS       = 0x09;
    const uint8_t MAGNETO_IDA          = 0x0A;  // should return 0x48
    const uint8_t MAGNETO_IDB          = 0x0B;  // should return 0x34
    const uint8_t MAGNETO_IDC          = 0x0C;  // should return 0x33

    // Set initial input parameters
    enum Ascale {
        AFS_2G = 0,
        AFS_4G,
        AFS_8G,
        AFS_16G
    };

    // Set accelerometer ODR and Bandwidth
    enum Arate {
        ARTBW_010_005 = 0, // 0.1 Hz ODR, 0.05Hz bandwidth
        ARTBW_020_010,
        ARTBW_039_020,
        ARTBW_078_039,
        ARTBW_156_078,
        ARTBW_313_156,
        ARTBW_125_625,
        ARTBW_25_125,
        ARTBW_50_25,
        ARTBW_100_50,
        ARTBW_200_100,
        ARTBW_400_200,
        ARTBW_800_400,
        ARTBW_1600_800,
        ARTBW_3200_1600  // 3200 Hz ODR, 1600 Hz bandwidth
    };

    enum Gscale {
        GFS_250DPS = 0,
        GFS_500DPS,
        GFS_1000DPS,
        GFS_2000DPS
    };

    enum Grate { // set gyro ODR and Bandwidth with 4 bits
        GRTBW_100_125 = 0, // 100 Hz ODR, 12.5 Hz bandwidth
        GRTBW_100_25,
        GRTBW_100_25a,
        GRTBW_100_25b,
        GRTBW_200_125,
        GRTBW_200_25,
        GRTBW_200_50,
        GRTBW_200_70,
        GRTBW_400_20,
        GRTBW_400_25,
        GRTBW_400_50,
        GRTBW_400_110,
        GRTBW_800_30,
        GRTBW_800_35,
        GRTBW_800_50,
        GRTBW_800_110  // 800 Hz ODR, 110 Hz bandwidth
    };

    enum Mrate { // set magnetometer ODR
        MRT_0075 = 0, // 0.75 Hz ODR
        MRT_015,      // 1.5 Hz
        MRT_030,      // 3.0 Hz
        MRT_075,      // 7.5 Hz
        MRT_15,       // 15 Hz
        MRT_30,       // 30 Hz
        MRT_75,       // 75 Hz ODR
    };

    uint8_t Gscale = GFS_250DPS;
    uint8_t Ascale = AFS_2G;
    uint8_t Arate = ARTBW_200_100; // 200 Hz ODR, 100 Hz bandwidth
    uint8_t Grate = GRTBW_200_50;  // 200 Hz ODR,  50 Hz bandwidth
    uint8_t Mrate = MRT_75;        //  75 Hz ODR
    float aRes, gRes, mRes; // scale resolutions per LSB for the sensors

    int16_t accelCount[3];  // 16-bit signed accelerometer sensor output
    int16_t gyroCount[3];   // 16-bit signed gyro sensor output
    int16_t magCount[3];    // 16-bit signed magnetometer sensor output
    float   magbias[3];     // User-specified magnetometer corrections values
};

#endif

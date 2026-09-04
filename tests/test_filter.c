#include "tm_filter.h"
#include "madgwick.h"
#include "imu_packet.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int checks;
#define CHECK(c) do { ++checks; if (!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(1); } } while (0)
static float dot(const float a[4],const float b[4]) {
    float d=0; for(int i=0;i<4;++i) d+=a[i]*b[i]; return d;
}
static void close_to(float a,float b,float tolerance) { CHECK(isfinite(a) && fabsf(a-b)<tolerance); }
static void unit(const float q[4]) { close_to(dot(q,q),1,3e-6f); }
static void same_rotation(const float a[4],const float b[4],float tolerance) {
    const float sign=dot(a,b)<0 ? -1.0f : 1.0f;
    for(int i=0;i<4;++i) close_to(a[i],sign*b[i],tolerance);
}
static uint32_t rng=12345;
static float random_value(void) {
    rng=1664525u*rng+1013904223u; return (float)(rng>>8)/8388608.0f-1.0f;
}
static void random_quaternion(float q[4]) {
    float sum=0; for(int i=0;i<4;++i) { q[i]=random_value(); sum+=q[i]*q[i]; }
    for(int i=0;i<4;++i) q[i]/=sqrtf(sum);
}
static void predicted_vertical(const float q[4],float a[3]) {
    a[0]=2*(q[1]*q[3]-q[0]*q[2]);
    a[1]=2*(q[0]*q[1]+q[2]*q[3]);
    a[2]=1-2*(q[1]*q[1]+q[2]*q[2]);
}

static void test_integration(void) {
    float e[4]={1,0,0,0}, f[4]={1,0,0,0};
    CHECK(tm_gyro_update_exact(e,1,0,0,1));
    CHECK(tm_gyro_update_fast(f,1,0,0,1));
    close_to(e[0],cosf(0.5f),1e-6f); close_to(e[1],sinf(0.5f),1e-6f);
    close_to(f[0],2/sqrtf(5),1e-6f); close_to(f[1],1/sqrtf(5),1e-6f);
    CHECK(fabsf(e[1]-f[1])>0.03f);
    for(int trial=0;trial<200;++trial) {
        float q[4],expected[4]; random_quaternion(q);
        const double gx=4*random_value(),gy=4*random_value(),gz=4*random_value();
        const float dt=0.001f+fabsf(random_value());
        const double n=sqrt(gx*gx+gy*gy+gz*gz),a=0.5*n*dt;
        /* Independent double-precision Hamilton product q * delta_q. */
        const double dw=cos(a),dx=sin(a)*gx/n,dy=sin(a)*gy/n,dz=sin(a)*gz/n;
        expected[0]=(float)(q[0]*dw-q[1]*dx-q[2]*dy-q[3]*dz);
        expected[1]=(float)(q[0]*dx+q[1]*dw+q[2]*dz-q[3]*dy);
        expected[2]=(float)(q[0]*dy-q[1]*dz+q[2]*dw+q[3]*dx);
        expected[3]=(float)(q[0]*dz+q[1]*dy-q[2]*dx+q[3]*dw);
        CHECK(tm_gyro_update_exact(q,(float)gx,(float)gy,(float)gz,dt));
        same_rotation(q,expected,6e-7f); unit(q);
    }
    float zero[4]={1,0,0,0};
    CHECK(tm_gyro_update_exact(zero,0,0,0,1)); close_to(zero[0],1,1e-7f);
    CHECK(tm_gyro_update_exact(zero,1e-9f,0,0,0.01f));
    close_to(zero[1],5e-12f,1e-17f);
    float before[4]; memcpy(before,zero,sizeof(zero));
    CHECK(!tm_gyro_update_fast(zero,NAN,0,0,1)); CHECK(memcmp(before,zero,sizeof(zero))==0);
    CHECK(!tm_gyro_update_exact(zero,0,0,0,0)); CHECK(memcmp(before,zero,sizeof(zero))==0);
    for(int i=0;i<10000;++i) CHECK(tm_gyro_update_fast(f,0.1f,-0.2f,0.3f,0.01f));
    unit(f);
}

static void test_accel_projection(void) {
    const float axes[][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},
                           {1e-8f,0,-1},{0,1e-8f,1}};
    for(int trial=0;trial<500;++trial) {
        float a[3],q[4],qa[4],vertical[3],original[3]; random_quaternion(q);
        if(trial<8) memcpy(a,axes[trial],sizeof(a));
        else for(int i=0;i<3;++i) a[i]=random_value();
        float n=hypotf(hypotf(a[0],a[1]),a[2]); for(int i=0;i<3;++i) a[i]/=n;
        CHECK(tm_accel_quaternion(q,a,qa)); unit(qa);
        predicted_vertical(qa,vertical);
        for(int i=0;i<3;++i) close_to(vertical[i],a[i],2e-6f);
        /* Optimal geodesic correction angle is angle between predicted and
         * measured verticals. This checks optimality without reusing the basis. */
        predicted_vertical(q,original);
        float cosine=original[0]*a[0]+original[1]*a[1]+original[2]*a[2];
        float d=dot(q,qa); close_to(d*d,0.5f*(1+cosine),2e-6f);
        float negative[4],negative_result[4]; for(int i=0;i<4;++i) negative[i]=-q[i];
        CHECK(tm_accel_quaternion(negative,a,negative_result));
        same_rotation(qa,negative_result,2e-6f);
    }
    float q[4]={1,0,0,0}, qa[4]={1,0,0,0}, zero[3]={0,0,0};
    CHECK(!tm_accel_quaternion(q,zero,qa)); CHECK(memcmp(q,qa,sizeof(q))==0);
    const float down[3]={0,0,-1}; CHECK(tm_accel_quaternion(q,down,qa)); unit(qa);
    float a[3]; predicted_vertical(qa,a); close_to(a[2],-1,1e-6f);
    const float opposite[4]={-1,0,0,0}; CHECK(tm_quaternion_blend(q,opposite,0.5f)); unit(q);
    close_to(q[0],1,1e-6f);
    CHECK(!tm_quaternion_blend(q,qa,-0.1f));
}

static void test_filter_gates(void) {
    tm_filter f; tm_filter_init(&f); CHECK(f.config.gyro_method==TM_GYRO_METHOD);
    CHECK(tm_filter_update(&f,0,0,0,0,0,TM_GRAVITY,0.01f)==1);
    CHECK(tm_filter_update(&f,0,0,0,0,0,TM_GRAVITY,0.01f)==1);
    CHECK(tm_filter_update(&f,0,0,0,0,0,-TM_GRAVITY,0.01f)==0); // Opposite vectors.
    CHECK(tm_filter_update(&f,0,0,0,0,0,20,0.01f)==0);
    CHECK(tm_filter_update(&f,0,0,1,0,0,0,0.01f)==0); CHECK(f.has_previous_accel==0);
    CHECK(f.q[3]>0); unit(f.q);
    CHECK(tm_filter_update(&f,0,0,1,NAN,0,0,0.01f)==0);
    tm_filter before=f;
    CHECK(tm_filter_update(&f,NAN,0,0,0,0,TM_GRAVITY,0.01f)==-1);
    CHECK(memcmp(&f,&before,sizeof(f))==0);
    f.config.fusion_gain=NAN; before=f;
    CHECK(tm_filter_update(&f,0,0,0,0,0,TM_GRAVITY,0.01f)==-1);
    CHECK(memcmp(&f,&before,sizeof(f))==0);
    tm_filter_init(&f);
    const float tilt[3]={0,0.6f*TM_GRAVITY,0.8f*TM_GRAVITY};
    for(int i=0;i<5000;++i) CHECK(tm_filter_update(&f,0,0,0,tilt[0],tilt[1],tilt[2],0.01f)==1);
    float a[3]; predicted_vertical(f.q,a); unit(f.q);
    close_to(a[1],0.6f,0.003f); close_to(a[2],0.8f,0.003f);
    tm_filter_reset(&f);
    CHECK(tm_filter_update(&f,0,0,0,0,0,TM_GRAVITY,0.01f)==1);
    float angle=0.003f;
    CHECK(tm_filter_update(&f,0,0,0,0,sinf(angle)*TM_GRAVITY,cosf(angle)*TM_GRAVITY,0.01f)==1);
    angle=0.009f;
    CHECK(tm_filter_update(&f,0,0,0,0,sinf(angle)*TM_GRAVITY,cosf(angle)*TM_GRAVITY,0.01f)==0);
}

static void test_madgwick_regressions(void) {
    float q[4]={1,0,0,0};
    CHECK(madgwick_update_imu(q,0,0,1,0,0,TM_GRAVITY,1,0.1f));
    close_to(q[3],1/sqrtf(5),1e-6f); // Zero gradient must not freeze yaw.
    float p[4]={1,0,0,0};
    CHECK(madgwick_update_imu(p,0,0,1,0,0,0,1,0.1f));
    same_rotation(q,p,1e-6f); // Zero accel must still integrate gyro.
}

static void put_u32(uint8_t *p,uint32_t x) { for(int i=0;i<4;++i) p[i]=(uint8_t)(x>>(8*i)); }
static void test_serial(void) {
    imu_decoder d={0}; imu_sample sample;
    uint8_t frame[IMU_PACKET_SIZE]={0xAA,0x55};
    const float values[6]={1.0f,-2.0f,9.80665f,90.0f,0.0f,-45.0f};
    for(int i=0;i<6;++i) { uint32_t bits; memcpy(&bits,&values[i],4); put_u32(frame+2+4*i,bits); }
    put_u32(frame+26,UINT32_MAX);
    const uint8_t noise[]={0,0xAA,0xAA,0,0x55,0x13};
    for(size_t i=0;i<sizeof(noise);++i) CHECK(!imu_decode_byte(&d,noise[i],&sample));
    for(int packet=0;packet<3;++packet) {
        for(int i=0;i<IMU_PACKET_SIZE;++i) CHECK(imu_decode_byte(&d,frame[i],&sample)==(i==IMU_PACKET_SIZE-1));
        CHECK(sample.sequence==-1);
        for(int i=0;i<3;++i) { close_to(sample.accel[i],values[i],1e-6f); close_to(sample.gyro[i],values[i+3],1e-6f); }
    }
    put_u32(frame+2,0x7FC00000u); // NaN payload rejected, then resynchronize.
    for(int i=0;i<IMU_PACKET_SIZE;++i) CHECK(!imu_decode_byte(&d,frame[i],&sample));
    put_u32(frame+2,0x3F800000u);
    int received=0; for(int i=0;i<IMU_PACKET_SIZE;++i) received+=imu_decode_byte(&d,frame[i],&sample);
    CHECK(received==1);
}
int main(void) {
    test_integration(); test_accel_projection(); test_filter_gates();
    test_madgwick_regressions(); test_serial();
    printf("PASS: %d checks (default gyro method: %s)\n",checks,
           TM_GYRO_METHOD==TM_GYRO_METHOD_EXACT ? "exact" : "fast");
    return 0;
}

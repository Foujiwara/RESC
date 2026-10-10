/* Diagnostic : tension vue par l'observateur vs tension reelle pendant l'I/f */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc.h"
#include "sim_motor.h"
static sim_motor_t M; static foc_adc_t ADC; static foc_pwm_t OUT;
void foc_hw_kill(void) { OUT.enable = false; }
int main(void){
    memset(&M,0,sizeof(M));
    M.r=0.06;M.ld=90e-6;M.lq=110e-6;M.lambda=0.018;M.pp=7;M.j=0.004;M.b=0.06;M.t_load=0.3;
    M.vbus=60;M.t_dead=400e-9;M.f_pwm=15000;
    M.i_lsb=-(3.44/4095.0)/((0.0005/3.0)*20.0);M.v_lsb=(3.44/4095.0)*(154700.0/4700.0);M.vph_lsb=M.v_lsb;
    M.off[0]=M.off[1]=M.off[2]=2048;sim_init(&M);
    foc_params_t p; memset(&p,0,sizeof p);
    p.f_pwm=15000;p.i_scale=M.i_lsb;p.v_scale=M.v_lsb;p.vph_scale=M.vph_lsb;p.cs_lowside=true;
    p.r=0.06;p.ld=90e-6;p.lq=110e-6;p.lambda=0.018;p.kp=0.1;p.ki=60;p.obs_gamma=1e3/(0.018*0.018);
    p.pll_kp=2000;p.pll_ki=30000;p.dt_comp=400e-9;p.m_max=0.95;p.t_win=3e-6;p.delay_comp=1;
    p.i_abs_trip=120;p.vbus_trip_max=90;p.vbus_trip_min=10;p.st_i=15;p.st_t_align=0.3;p.st_t_ramp=1;
    p.st_omega=1500*2*M_PI/60;p.st_omega_exit=700*2*M_PI/60;p.st_timeout=4;p.i_slew=2000;
    foc_init(&p);
    for(int k=0;k<6000;k++){foc_isr(&ADC,&OUT);sim_step(&M,&OUT,&ADC);}
    g_foc_cmd.iq_ref=20;g_foc_cmd.dir=1;g_foc_cmd.allow_start=1;g_foc_cmd.run=1;
    double d_prev[3]={.5,.5,.5}, d_prev2[3]={.5,.5,.5};
    for(int k=0;k<30000;k++){
        foc_isr(&ADC,&OUT);
        /* tension reelle appliquee pendant l'intervalle qui vient d'etre echantillonne */
        double i[3]; sim_phase_currents(&M,i);
        if(k%1500==0){
            double th=sim_theta_e(&M);
            printf("t=%.2f st=%d we=%7.1f obs(va=%7.2f vb=%7.2f) ia_obs=%6.2f ia_true=%6.2f ib_obs=%6.2f flux=%.2f th_true=%6.1f th_obs=%6.1f\n",
              k/15000.0, foc_state(), M.omega_m*M.pp, g_foc_telem.dbg_va, g_foc_telem.dbg_vb, g_foc_telem.dbg_ia, i[0],
              g_foc_telem.dbg_ib, g_foc_telem.flux_ratio, fmod(th+10*M_PI,2*M_PI)*180/M_PI-180, g_foc_telem.theta*180/M_PI);
        }
        sim_step(&M,&OUT,&ADC);
    }
}

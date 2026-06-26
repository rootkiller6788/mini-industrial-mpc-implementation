#include "mpc_common.h"
#include <stdio.h>

int main(void) {
    mpc_tuning_t tuning;
    mpc_tuning_init_default(&tuning);
    printf("np=%d nc=%d ts=%.2f\n", tuning.np, tuning.nc, tuning.ts);
    printf("q[0]=%.2f r[0]=%.2f\n", tuning.q_weight[0], tuning.r_weight[0]);
    
    mpc_ss_model_t model;
    mpc_model_init(&model, 2, 1, 1, 0);
    printf("nx=%d nu=%d ny=%d\n", model.nx, model.nu, model.ny);
    
    printf("MINIMAL TEST PASSED\n");
    return 0;
}

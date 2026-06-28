/* lqr_gains.h — 5-term LQR gains, offline computed */
/* Cost: J = e_y^2 + 0.5*e_theta^2 + 0.05*omega^2, wo=50, alpha=0.3 */
#ifndef LQR_GAINS_H
#define LQR_GAINS_H
#include "common.h"

#define LQR_SPEED_POINTS 7

static const f32 lqr_speed_bp[] = {0.1f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f};
static const f32 lqr_gains[7][5] = {
    {4.472136f,10.817448f,2.524071f,0.071912f,2.189550f},  /* v=0.1 */
    {4.472136f,2.502834f,2.919973f,0.082413f,5.241288f},  /* v=0.5 */
    {4.472136f,1.381378f,3.223215f,0.090075f,7.749181f},  /* v=1.0 */
    {4.472136f,0.988300f,3.459050f,0.095818f,9.788679f},  /* v=1.5 */
    {4.472136f,0.784238f,3.659776f,0.100565f,11.579592f},  /* v=2.0 */
    {4.472136f,0.657938f,3.837971f,0.104674f,13.208215f},  /* v=2.5 */
    {4.472136f,0.571444f,4.000110f,0.108330f,14.719347f}  /* v=3.0 */
};

#endif

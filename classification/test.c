#include "analyze.h"
#include "stdio.h"
#include "average.h"
#include "integral.h"
#include "integral_3d.h"
#include "integral_update_3d.h"
#include "adjust_acceleration.h"
#include "vector_component.h"
#include "../commons/constants.h"
#include "../commons/data_formats.h"


int main(int argc, char *argv[]) {
	
	{
		double values[] = {1, 2, 3, 4, 5};
		AverageState state;
		init_analysis_functor(&state.base, 1, "average");
		init_average_state(&state);
		for (int i = 0; i < 5; i++) {
			average(&state, values[i]);
			printf("\n");
		}
		printf("Average: %f, N_samples %d \n", state.value, state.n_samples);
		printf("\n");
	}

	{
		double values[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, -50, -51, -52, -53, -54, -55, -56, -57, -58, -59, -60, -61, -62, -63, -64, -65, -66, -67, -68, -69, -70, -71, -72, -73, -74, -75, -76, -77, -78, -79, -80, -81, -82, -83, -84, -85, -86, -87, -88, -89, -90, -91, -92, -93, -94, -95, -96, -97, -98, -99};
		IntegralState integral_state;
		init_analysis_functor(&integral_state.base, 1, "integral");
		init_integral_state(&integral_state, SAMPLE_PERIOD);
		for (int i = 0; i < 100; i++) {
			integral(&integral_state, values[i]);
			printf("\n");
		}
		printf("Integral: %f, N_samples %d \n", integral_state.value, integral_state.n_samples);
		printf("\n");
	}

	{
		Vec3d values[] = {
			{.x = 1, .y = 1, .z = -1},
			{.x = 2, .y = 2, .z = -2},
			{.x = 3, .y = 3, .z = -3},
			{.x = 4, .y = 4, .z = -4},
			{.x = 5, .y = 5, .z = -5}
		};
		Integral3dState integral_state;
		init_analysis_functor(&integral_state.base, 1, "integral_3d");
		init_integral_3d_state(&integral_state, 1);
		for (int i = 0; i < 5; i++) {
			integral_3d(&integral_state, values[i]);
			printf("\n");
		}
		printf("Integral 3D: %f, %f, %f, N_samples %d \n", integral_state.value.x, integral_state.value.y, integral_state.value.z, integral_state.x.n_samples);
		printf("\n");
	}

	{
		Vec3d values[] = {
			{.x = 1, .y = 1, .z = -1},
			{.x = 2, .y = 2, .z = -2},
			{.x = 3, .y = 3, .z = -3},
			{.x = 4, .y = 4, .z = -4},
			{.x = 5, .y = 5, .z = -5}
		};
		IntegralUpdate3dState integral_update_state;
		init_analysis_functor(&integral_update_state.base, 1, "integral_update_3d");
		init_integral_update_3d_state(&integral_update_state, 1);
		for (int i = 0; i < 5; i++) {
			integral_update_3d(&integral_update_state, values[i]);
			printf("\n");
		}
		printf("Integral Update 3D: %f, %f, %f, N_samples %d \n", integral_update_state.value.x, integral_update_state.value.y, integral_update_state.value.z, integral_update_state.x.n_samples);
		printf("\n");
	}


	/* Sample from python.
		           time        ax        ay        az        gx        gy        gz  \
		Data set                                                                      
		0         13542 -2.851494 -2.366668  9.566033  0.000000  0.000000  0.000000   
		1         13677 -2.873042 -2.331952  9.624092  0.000000  0.000000  0.000000   
		2         13813 -2.883816 -2.209848  9.517550  0.000000  0.000000  0.000000   
		3         13948 -2.948459 -2.255338  9.443330  0.000000  0.000000  0.000000   
		4         14083 -2.756325 -2.263119  9.430760  0.000000  0.000000  0.000000   
		5         14219 -2.792238 -2.132635  9.355343  0.007507 -0.005859 -0.004700   
		6         14354 -2.805406 -2.013524  9.230845  0.007812 -0.006042 -0.004272   
		7         14490 -2.800019 -2.062605  9.185953  0.009155 -0.006409 -0.003601   
		8         14625 -2.850896 -2.227805  9.183559  0.010254 -0.004883 -0.002014   
		9         14760 -2.823961 -2.297236  9.060258  0.010803 -0.003235 -0.000061   
	*/

	{
		BMI2SensAxisData values[] = {
			{.x = -4764, .y = -3954, .z = 15982},
			{.x = -4800, .y = -3896, .z = 16079},
			{.x = -4818, .y = -3692, .z = 15901},
			{.x = -4926, .y = -3768, .z = 15777},
			{.x = -4605, .y = -3781, .z = 15756},
			{.x = -4665, .y = -3563, .z = 15630},
			{.x = -4687, .y = -3364, .z = 15422},
			{.x = -4678, .y = -3446, .z = 15347},
			{.x = -4763, .y = -3722, .z = 15343},
			{.x = -4718, .y = -3838, .z = 15137},
		};

		AdjustAccelerationState acc_adjusted;
		init_analysis_functor(&acc_adjusted.base, 1, "acc_adjusted");
		init_adjust_acceleration_state(&acc_adjusted);
		for (int i = 0; i < 10; i++) {
			adjust_acceleration(&acc_adjusted, values[i]);
			printf("\n");
		}
		printf("Adjusted acceleration: %f, %f, %f \n", acc_adjusted.value.x, acc_adjusted.value.y, acc_adjusted.value.z);
		printf("\n");
	}

	{
		Vec3d g = {.x = 0, .y = 1, .z = 9.8};
		Vec3d a = {.x = 1, .y = 1, .z = -1};
		// a - g = <1, 0, -10.8>
		// y = (a - g) . g = 1*0 + 0*1 + -10.8*9.8 = -105.84
		// |g| = sqrt(0**2 + 1**2 + 9.8**2) = 9.850888284819803
		// y / |g| = -10.74420873933767
		VectorComponentState a_sub_onto_g;
		init_analysis_functor(&a_sub_onto_g.base, 1, "a_sub_onto_g");
        vector_component(&a_sub_onto_g, a, g);
		printf("\n");
		printf("Vector component: %f \n", a_sub_onto_g.value);
	}

	{
		BMI2SensData test_data[] = {
			{.acc = {.x = -4485, .y = -3720, .z = 14912}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1354},
			{.acc = {.x = -4596, .y = -3630, .z = 15011}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1367},
			{.acc = {.x = -4614, .y = -3429, .z = 14823}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1381},
			{.acc = {.x = -4704, .y = -3499, .z = 14673}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1394},
			{.acc = {.x = -4410, .y = -3516, .z = 14628}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1408},
			{.acc = {.x = -4467, .y = -3309, .z = 14540}, .gyr = {.x = 12, .y = -9, .z = -7}, .sens_time = 1421},
			{.acc = {.x = -4488, .y = -3134, .z = 14328}, .gyr = {.x = 12, .y = -10, .z = -7}, .sens_time = 1435},
			{.acc = {.x = -4481, .y = -3209, .z = 14235}, .gyr = {.x = 14, .y = -10, .z = -6}, .sens_time = 1449},
			{.acc = {.x = -4553, .y = -3468, .z = 14228}, .gyr = {.x = 16, .y = -9, .z = -4}, .sens_time = 1462},
			{.acc = {.x = -4502, .y = -3566, .z = 14057}, .gyr = {.x = 17, .y = -6, .z = 0}, .sens_time = 1476},
			{.acc = {.x = -4485, .y = -3720, .z = 14912}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1354},
			{.acc = {.x = -4596, .y = -3630, .z = 15011}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1367},
			{.acc = {.x = -4614, .y = -3429, .z = 14823}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1381},
			{.acc = {.x = -4704, .y = -3499, .z = 14673}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1394},
			{.acc = {.x = -4410, .y = -3516, .z = 14628}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1408},
			{.acc = {.x = -4467, .y = -3309, .z = 14540}, .gyr = {.x = 12, .y = -9, .z = -7}, .sens_time = 1421},
			{.acc = {.x = -4488, .y = -3134, .z = 14328}, .gyr = {.x = 12, .y = -10, .z = -7}, .sens_time = 1435},
			{.acc = {.x = -4481, .y = -3209, .z = 14235}, .gyr = {.x = 14, .y = -10, .z = -6}, .sens_time = 1449},
			{.acc = {.x = -4553, .y = -3468, .z = 14228}, .gyr = {.x = 16, .y = -9, .z = -4}, .sens_time = 1462},
			{.acc = {.x = -4502, .y = -3566, .z = 14057}, .gyr = {.x = 17, .y = -6, .z = 0}, .sens_time = 1476},
			{.acc = {.x = -4485, .y = -3720, .z = 14912}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1354},
			{.acc = {.x = -4596*4, .y = -3630, .z = 15011}, .gyr = {.x =10000, .y = 0, .z = 0}, .sens_time = 1367},
			{.acc = {.x = -4614*4, .y = -3429, .z = 14823}, .gyr = {.x =10000, .y = 0, .z = 0}, .sens_time = 1381},
			{.acc = {.x = -4704*4, .y = -3499, .z = 14673}, .gyr = {.x =10000, .y = 0, .z = 0}, .sens_time = 1394},
			{.acc = {.x = -4410*4, .y = -3516, .z = 14628}, .gyr = {.x =10000, .y = 0, .z = 0}, .sens_time = 1408},
			{.acc = {.x = -4467*4, .y = -3309, .z = 14540}, .gyr = {.x = 10000, .y = -9, .z = -7}, .sens_time = 1421},
			{.acc = {.x = -4488*4, .y = -3134, .z = 14328}, .gyr = {.x = 10000, .y = -10, .z = -7}, .sens_time = 1435},
			{.acc = {.x = -4481*4, .y = -3209, .z = 14235}, .gyr = {.x = 10000, .y = -10, .z = -6}, .sens_time = 1449},
			{.acc = {.x = -4553*4, .y = -3468, .z = 14228}, .gyr = {.x = 16, .y = -9, .z = -4}, .sens_time = 1462},
			{.acc = {.x = -4502*4, .y = -3566, .z = 14057}, .gyr = {.x = 17, .y = -6, .z = 0}, .sens_time = 1476},
			{.acc = {.x = -4485*4, .y = -3720, .z = 14912}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1354},
			{.acc = {.x = -4596*4, .y = -3630, .z = 15011}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1367},
			{.acc = {.x = -4614*4, .y = -3429, .z = 14823}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1381},
			{.acc = {.x = -4704*4, .y = -3499, .z = 14673}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1394},
			{.acc = {.x = -4410*4, .y = -3516, .z = 14628}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1408},
			{.acc = {.x = -4467*4, .y = -3309, .z = 14540}, .gyr = {.x = -10000, .y = -9, .z = -7}, .sens_time = 1421},
			{.acc = {.x = -4488*4, .y = -3134, .z = 14328}, .gyr = {.x = -10000, .y = -10, .z = -7}, .sens_time = 1435},
			{.acc = {.x = -4481*2, .y = -3209, .z = 14235}, .gyr = {.x = -10000, .y = -10, .z = -6}, .sens_time = 1449},
			{.acc = {.x = -4553*2, .y = -3468, .z = 14228}, .gyr = {.x = -10000, .y = -9, .z = -4}, .sens_time = 1462},
			{.acc = {.x = -4502*2, .y = -3566, .z = 14057}, .gyr = {.x = -10000, .y = -6, .z = 0}, .sens_time = 1476},
			{.acc = {.x = -4485*2, .y = -3720, .z = 14912}, .gyr = {.x =- 10000, .y = 0, .z = 0}, .sens_time = 1354},
			{.acc = {.x = -4596*2, .y = -3630, .z = 15011}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1367},
			{.acc = {.x = -4614*2, .y = -3429, .z = 14823}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1381},
			{.acc = {.x = -4704*2, .y = -3499, .z = 14673}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1394},
			{.acc = {.x = -4410*2, .y = -3516, .z = 14628}, .gyr = {.x = 0, .y = 0, .z = 0}, .sens_time = 1408},
			{.acc = {.x = -4467*2, .y = -3309, .z = 14540}, .gyr = {.x = 12, .y = -9, .z = -7}, .sens_time = 1421},
			{.acc = {.x = -4488*2, .y = -3134, .z = 14328}, .gyr = {.x = 12, .y = -10, .z = -7}, .sens_time = 1435},
			{.acc = {.x = -4481*2, .y = -3209, .z = 14235}, .gyr = {.x = 14, .y = -10, .z = -6}, .sens_time = 1449},
			{.acc = {.x = -4553*2, .y = -3468, .z = 14228}, .gyr = {.x = 16, .y = -9, .z = -4}, .sens_time = 1462},
			{.acc = {.x = -4502*2, .y = -3566, .z = 14057}, .gyr = {.x = 17, .y = -6, .z = 0}, .sens_time = 1476},
			{.acc = {.x = -4485*2, .y = -3720, .z = 14912}, .gyr = {.x = 0, .y = 0, .z = 10000}, .sens_time = 1354},
			{.acc = {.x = -4596*2, .y = -3630, .z = 15011}, .gyr = {.x = 0, .y = 0, .z = 10000}, .sens_time = 1367},
			{.acc = {.x = -4614*2, .y = -3429, .z = 14823}, .gyr = {.x = 0, .y = 0, .z = 10000}, .sens_time = 1381},
			{.acc = {.x = -4704*2, .y = -3499, .z = 14673}, .gyr = {.x = 0, .y = 0, .z = 10000}, .sens_time = 1394},
			{.acc = {.x = -4410*2, .y = -3516, .z = 14628}, .gyr = {.x = 0, .y = 0, .z = 10000}, .sens_time = 1408},
			{.acc = {.x = -4467*2, .y = -3309, .z = 14540}, .gyr = {.x = 12, .y = -9, .z = -7}, .sens_time = 1421},
			{.acc = {.x = -4488*2, .y = -3134, .z = 14328}, .gyr = {.x = 12, .y = -10, .z = -7}, .sens_time = 1435},
			{.acc = {.x = -4481*2, .y = -3209, .z = 14235}, .gyr = {.x = 14, .y = -10, .z = -6}, .sens_time = 1449},
			{.acc = {.x = -4553*2, .y = -3468, .z = 14228}, .gyr = {.x = 16, .y = -9, .z = -4}, .sens_time = 1462},
			{.acc = {.x = -4502*2, .y = -3566, .z = 14057}, .gyr = {.x = 17, .y = -6, .z = 0}, .sens_time = 1476},
			{.acc = {.x = -4485*2, .y = -3720, .z = 14912}, .gyr = {.x = 0, .y = 0, .z = -10000}, .sens_time = 1354},
			{.acc = {.x = -4596*2, .y = -3630, .z = 15011}, .gyr = {.x = 0, .y = 0, .z = -10000}, .sens_time = 1367},
			{.acc = {.x = -4614*2, .y = -3429, .z = 14823}, .gyr = {.x = 0, .y = 0, .z = -10000}, .sens_time = 1381},
			{.acc = {.x = -4704*2, .y = -3499, .z = 14673}, .gyr = {.x = 0, .y = 0, .z = -10000}, .sens_time = 1394},
			{.acc = {.x = -4410*2, .y = -3516, .z = 14628}, .gyr = {.x = 0, .y = 0, .z = -10000}, .sens_time = 1408},
			{.acc = {.x = -4467*2, .y = -3309, .z = 14540}, .gyr = {.x = 12, .y = -9, .z = -7}, .sens_time = 1421},
			{.acc = {.x = -4488*2, .y = -3134, .z = 14328}, .gyr = {.x = 12, .y = -10, .z = -7}, .sens_time = 1435},
			{.acc = {.x = -4481*2, .y = -3209, .z = 14235}, .gyr = {.x = 14, .y = -10, .z = -6}, .sens_time = 1449},
			{.acc = {.x = -4553*2, .y = -3468, .z = 14228}, .gyr = {.x = 16, .y = -9, .z = -4}, .sens_time = 1462},
			{.acc = {.x = -4502*2, .y = -3566, .z = 14057}, .gyr = {.x = 17, .y = -6, .z = 0}, .sens_time = 1476},
		};

		AnalysisResult result;
		AnalysisStatus status = analyze(test_data, 205 - 136, test_data[0], 1, &result);
		printf("\nAnalysis result: %d \n", result.classification);
	}

	return 0;
}
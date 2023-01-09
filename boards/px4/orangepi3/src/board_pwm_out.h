#pragma once

#include <px4_platform/pwm_out_base.h>

#define BOARD_PWM_OUT_IMPL NavioSysfsPWMOut

namespace pwm_out
{

/**
 ** class NavioSysfsPWMOut
 * PWM output class for Navio Sysfs
 */
class NavioSysfsPWMOut : public PWMOutBase
{
public:
	NavioSysfsPWMOut(int max_num_outputs);
	virtual ~NavioSysfsPWMOut();

	int init() override;

	int send_output_pwm(const uint16_t *pwm, int num_outputs) override;

private:
	int pwm_write_sysfs(char *path, int value);
	int pwm_write_sysfs_string(char *path, char *str);

	static const int MAX_NUM_PWM = 4;
	static const int FREQUENCY_PWM = 400;
	int pwm_pin_map[4] = {2,3,6,7};
	int _pwm_fd[MAX_NUM_PWM];
	int _pwm_num;

	static const char _device[];
};

}

from gym.envs.registration import register
register(
	id='f110-v0',
	entry_point='f110_gym.envs:F110Env',
	)

register(
	id='f110rl-v0',
	entry_point='f110_gym.envs:RLF110Env',
	)

register(
	id='f110direct-v0',
	entry_point='f110_gym.envs:F110EnvDirect',
	)

register(
	id='f110rldirect-v0',
	entry_point='f110_gym.envs:RLF110EnvDirect',
	)

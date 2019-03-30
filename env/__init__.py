from gym.envs.registration import register

# all gym environment are wrapped by a time_limit wrapper provided by OpenAI gym automatically
# time_limit wrapper is the first layer wrapper
register(
    id='icub-v0',
    entry_point='env.Gazebo.icub_env:icubEnv',
)

register(
    id='Reacher-v0',
    entry_point='env.roboschool.reacher_env:Reacher',
    max_episode_steps=150,
    reward_threshold=18.0,
    tags={ "pg_complexity": 1*1000000 },
    )

register(
	id='ReacherPyBulletEnv-v0',
	entry_point='env.pybullet.Extra.gym_manipulator_envs:ReacherBulletEnv',
	max_episode_steps=150,
	reward_threshold=18.0,
	)

register(
	id='ReacherBulletPositionEnv-v0',
	entry_point='env.pybullet.Extra.gym_manipulator_envs:ReacherBulletPositionEnv',
	max_episode_steps=150,
	reward_threshold=18.0,
	)

register(
	id='KukaGrasping-v0',
	entry_point='env.pybullet.Extra.gym_manipulator_envs:KukaGrasping',
	timestep_limit=1000,
	reward_threshold=5.0,
)

register(
	id='KukaTouching-v0',
	entry_point='env.pybullet.Extra.gym_manipulator_envs:KukaTouching',
	timestep_limit=300,
	reward_threshold=5.0,
)



register(
	id='iCubPyBulletEnv-v0',
	entry_point='env.pybullet.gym_manipulator_envs:iCubBulletEnv',
	max_episode_steps=150,
	reward_threshold=18.0,
	)

from raya.application_base import RayaApplicationBase
from raya.skills import RayaSkillHandler
from skills.skill_belinson_approach.skill_belinson_approach import SkillBelinsonApproach

class RayaApplication(RayaApplicationBase):

    async def setup(self):
        self.log.warn(f'Registering skill')
        self.skill_dock:RayaSkillHandler = self.register_skill(SkillBelinsonApproach)
        self.log.warn('Executing setup')
        await self.skill_dock.execute_setup(
            setup_args={
                'map_name' : 'Belinson__hospital01',
                'only_face' : False
            }
        )

    async def loop(self):
        self.log.warn(f'Executing skill')
        try:
            execute_result = await self.skill_dock.execute_main(
                execute_args={
                    'face_angle' : -16.0
                },
                callback_feedback=self.cb_feedback
            )
            self.log.debug(f'result: {execute_result}')
        except Exception as error:
            self.log.error(f'Error executing skill: {error}')
        finally:
            self.finish_app()


    async def finish(self):
        self.log.warn(f'Finishing skill')
        # await self.skill_dock.execute_finish(
        #     callback_feedback=self.cb_feedback
        # )
        self.log.warn(f'App finished')


    async def cb_feedback(self, feedback):
        self.log.debug(feedback)
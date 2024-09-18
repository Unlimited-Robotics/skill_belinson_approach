from raya.application_base import RayaApplicationBase


class RayaApplication(RayaApplicationBase):

    async def setup(self):
        # Create local attributes and variables
        self.i = 0
        self.log.info(f'Hello from setup()')

    async def loop(self):
        # Loop
        self.i += 1
        self.log.info(f'Hello from loop(), i={self.i}')
        await self.sleep(0.2)
        if self.i >= 10:
            self.finish_app()

    async def finish(self):
        # Finishing instructions
        self.log.warn(f'Hello from finish()')

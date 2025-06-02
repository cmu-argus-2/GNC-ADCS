import gc
import traceback

# from core import logger


class TemplateTask:
    """
    A Task Object.

    Attributes:
        ID:          Unique identifier for the task.
        name:        Name of the task object.
    """

    def __init__(self, id):
        self.ID = id
        self.name = "TASK"
        self.frequency = None

    def set_frequency(self, frequency):
        """
        Set the frequency of the task

        :param frequency: Frequency of the task
        """
        self.frequency = frequency

    async def main_task(self, *args, **kwargs):
        """
        Contains the code for the user defined task.

        :param `*args`: Variable number of arguments used for task execution.
        :param `**kwargs`: Variable number of keyword arguments used for task execution.
        """
        pass

    async def _run(self):
        """
        Try to run the main task, then call handle_error if an error is raised.
        """
        try:
            # gc.collect()
            await self.main_task()
            gc.collect()
        except Exception as e:
            self.debug(e, "".join(traceback.format_exception(e)))

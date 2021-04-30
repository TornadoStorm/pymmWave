import abc
from abc import ABC, abstractmethod
from typing import Any

class Logger(ABC, metaclass=abc.ABCMeta):
    def __init__(self) -> None: ...
    @abstractmethod
    def log(self, *args: Any, **kwargs: Any) -> None: ...
    @abstractmethod
    def error(self, *args: Any, **kwargs: Any) -> None: ...

class StdOutLogger(Logger):
    def log(self, *args: Any, **kwargs: Any) -> None: ...
    def error(self, *args: Any, **kwargs: Any) -> None: ...

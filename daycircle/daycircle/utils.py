"""
daycircle.utils: utility functions and data structures for daycircle
--------------------------------------------------------------------
by mark <mark@joshwel.co>

This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>
"""

from functools import wraps
from typing import Callable, Generic, NamedTuple, ParamSpec, TypeVar

# result class yoinked from https://github.com/markjoshwel/surplus
ResultType = TypeVar("ResultType")


class Result(NamedTuple, Generic[ResultType]):
    """
    typing.NamedTuple representing a result for safe value retrieval

    arguments
        value: ResultType
            value to return or fallback value if erroneous
        error: BaseException | None = None
            exception if any

    methods
        def __bool__(self) -> bool: ...
        def get(self) -> ResultType: ...
        def cry(self, string: bool = False) -> str: ...

    example
        # do something
        def some_operation(path) -> Result[str]:
            try:
                file = open(path)
                contents = file.read()

            except Exception as exc:
                # must pass a default value
                return Result[str]("", error=exc)

            else:
                return Result[str](contents)

        # call function and handle result
        result = some_operation("some_file.txt")

        if not result:  # check if the result is erroneous
            # .cry() raises the exception
            # (or returns it as a string error message using string=True)
            result.cry()
            ...

        else:
            # .get() raises exception or returns value,
            # but since we checked for errors this is safe
            print(result.get())
    """

    value: ResultType
    error: BaseException | None = None

    def __bool__(self) -> bool:
        """method that returns True if self.error is not None"""
        return self.error is None

    def cry(self, string: bool = False) -> str:
        """
        method that raises self.error if is an instance of BaseException,
        returns self.error if is an instance of str, or returns an empty string if
        self.error is None

        arguments
            string: bool = False
                if self.error is an Exception, returns it as a string error message
        """

        if isinstance(self.error, BaseException):
            if string:
                message = f"{self.error}"
                name = self.error.__class__.__name__
                return f"{message} ({name})" if (message != "") else name

            raise self.error

        if isinstance(self.error, str):
            return self.error

        return ""

    def get(self) -> ResultType:
        """method that returns self.value if Result is non-erroneous else raises error"""
        if isinstance(self.error, BaseException):
            raise self.error
        return self.value


P = ParamSpec("P")
R = TypeVar("R")


def result(default: R) -> Callable[[Callable[P, R]], Callable[P, Result[R]]]:
    """decorator that wraps a non-Result-returning function to return a Result"""

    def result_decorator(func: Callable[P, R]) -> Callable[P, Result[R]]:
        @wraps(func)
        def wrapper(*args: P.args, **kwargs: P.kwargs) -> Result[R]:
            try:
                return Result(func(*args, **kwargs))
            except Exception as exc:
                return Result(default, error=exc)

        return wrapper

    return result_decorator

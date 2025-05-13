class DataResult:
    def __init__(self, data:str=None, error:str=None):
        self.data = data
        self.error = error

    def __str__(self):
        return f"data:{self.data},error:{self.error}"


class SuccessDataResult(DataResult):
    def __init__(self, data:str=None):
        super().__init__(data=data)

    def __str__(self):
        return f"data:{self.data},error:{self.error}"

class ErrorDataResult(DataResult):
    def __init__(self, error:str=None):
        super().__init__(error=error)
    
    def __str__(self):
        return f"data:{self.data},error:{self.error}"

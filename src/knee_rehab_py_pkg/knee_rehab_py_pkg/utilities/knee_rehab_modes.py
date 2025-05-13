class KneeRehabModes:
    _mode_names = {
        1: "TRAINING_MODE",
        2: "EXERCISE_MODE",
        3: "FREE_MODE"
    }
    
    _status_names = {
        0: "MODE_STOP",
        1: "MODE_START"
    }
    
    class Modes:
        TRAINING_MODE = 1
        EXERCISE_MODE = 2
        FREE_MODE = 3

    class Status:   
        MODE_STOP = 0
        MODE_START = 1

    @classmethod
    def get_mode_name(cls, mode_number:int) -> str:
        return cls._mode_names.get(mode_number, "UNKNOWN_MODE")
    
    @classmethod
    def get_status_name(cls, status_number:int) -> str:
        return cls._status_names.get(status_number, "UNKNOWN_STATUS")

    @classmethod
    def get_mode_id(cls, mode_name:str) -> int:
        for index, mode in cls._mode_names.items():
            if mode == mode_name:
                return index
        return -1
    
    @classmethod
    def get_status_id(cls, status_name:str) -> int:
        for index, status in cls._status_names.items():
            if status == status_name:
                return index
        return -1
    
        
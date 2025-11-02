import logging

class DWALogging(logging.Logger):

    
    def __init__(self, node):
        super().__init__("root")
        
        # Workaround #
        LoggingSeverity = type(node.get_logger().get_effective_level())

        # Map between levels in ROS2 and python's logging module
        severity_map_ros = {
            LoggingSeverity.DEBUG: logging.DEBUG,
            LoggingSeverity.INFO: logging.INFO,
            LoggingSeverity.WARN: logging.WARN,
            LoggingSeverity.ERROR: logging.ERROR,
            LoggingSeverity.FATAL: logging.FATAL,
        }

        # Add a custom handler which will intercept messages from python logging and send them through the node's logger
        self.addHandler(DWALoggingHandler(node))
        # Set the python logger's level from the node's severity. If this isn't set correctly some messages will be
        # missed by the handler
        self.setLevel(severity_map_ros[node.get_logger().get_effective_level()])


class DWALoggingHandler(logging.Handler):

    def __init__(self, node):
        super().__init__()
        self.node_logger = node.get_logger()

    def emit(self, record):
        # Check the level of the record in relation to the log levels, and use the corresponding function of the
        # node's logger
        if record.levelno >= logging.FATAL:
            self.node_logger.fatal(record.msg)
        elif record.levelno >= logging.ERROR:
            self.node_logger.error(record.msg)
        elif record.levelno >= logging.WARNING:
            self.node_logger.warning(record.msg)
        elif record.levelno >= logging.INFO:
            self.node_logger.info(record.msg)
        else:
            self.node_logger.debug(record.msg)


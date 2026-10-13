"""Connection values are supplied by CLI or environment; importing is offline."""
import os
SERIAL_PORT=os.environ.get("PI_CONSOLE_PORT", "")
SERIAL_BAUD=115200
SCOPE_VISA_ADDRESS=os.environ.get("PI_SCOPE_ADDRESS", "")
SCOPE_VISA_LIBRARY=os.environ.get("PI_VISA_LIBRARY", "")
SCOPE_VISA_DLL_DIR=os.environ.get("PI_VISA_DLL_DIR", "")

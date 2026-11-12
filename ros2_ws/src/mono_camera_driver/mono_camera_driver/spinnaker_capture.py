"""USB3 Vision monochrome acquisition through Teledyne Spinnaker / PySpin."""


class SpinnakerCapture:
    def __init__(self, serial: str = '', timeout_ms: int = 1000) -> None:
        try:
            import PySpin
        except ImportError as exc:
            raise RuntimeError(
                'PySpin is missing. Install the ARM64 Ubuntu 22.04 Spinnaker SDK '
                'and its matching Python 3.10 wheel from Teledyne.'
            ) from exc
        self.pyspin = PySpin
        self.timeout_ms = timeout_ms
        self.system = PySpin.System.GetInstance()
        self.camera_list = self.system.GetCameras()
        self.camera = None
        self.started = False
        self.initialized = False
        try:
            count = self.camera_list.GetSize()
            if count == 0:
                raise RuntimeError('Spinnaker found no camera; check USB3 cable, power, and permissions')
            for index in range(count):
                camera = self.camera_list.GetByIndex(index)
                if not serial or str(camera.TLDevice.DeviceSerialNumber.GetValue()) == serial:
                    self.camera = camera
                    break
            if self.camera is None:
                raise RuntimeError(f'Spinnaker did not find camera serial {serial!r}')
            self.camera.Init()
            self.initialized = True
            self.camera.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
            self.processor = PySpin.ImageProcessor()
            self.camera.BeginAcquisition()
            self.started = True
        except Exception:
            self.release()
            raise

    def isOpened(self) -> bool:
        return self.started

    def read(self):
        try:
            image = self.camera.GetNextImage(self.timeout_ms)
        except self.pyspin.SpinnakerException:
            return False, None
        try:
            if image.IsIncomplete():
                return False, None
            if image.GetPixelFormat() == self.pyspin.PixelFormat_Mono8:
                frame = image.GetNDArray().copy()
            else:
                converted = self.processor.Convert(image, self.pyspin.PixelFormat_Mono8)
                frame = converted.GetNDArray().copy()
            return True, frame
        finally:
            image.Release()

    def release(self) -> None:
        if self.started:
            self.camera.EndAcquisition()
            self.started = False
        if self.initialized:
            self.camera.DeInit()
            self.initialized = False
        self.camera = None
        if self.camera_list is not None:
            self.camera_list.Clear()
            self.camera_list = None
        if self.system is not None:
            self.system.ReleaseInstance()
            self.system = None

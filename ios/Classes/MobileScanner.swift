//
//  MobileScanner.swift
//  mobile_scanner
//
//  Created by Julian Steenbakker on 15/02/2022.
//

import Foundation
import Flutter
import AVFoundation
import MLKitVision
import MLKitBarcodeScanning

typealias MobileScannerCallback = ((Array<Barcode>?, Error?, UIImage) -> ())
typealias TorchModeChangeCallback = ((Int?) -> ())
typealias ZoomScaleChangeCallback = ((Double?) -> ())

public class MobileScanner: NSObject, AVCaptureVideoDataOutputSampleBufferDelegate, FlutterTexture {
    /// Capture session of the camera
    var captureSession: AVCaptureSession?

    /// The selected camera
    var device: AVCaptureDevice!

    /// The long lived barcode scanner for scanning barcodes from a camera preview.
    var scanner: BarcodeScanner? = nil

    /// Default position of camera
    var videoPosition: AVCaptureDevice.Position = AVCaptureDevice.Position.back

    /// When results are found, this callback will be called
    let mobileScannerCallback: MobileScannerCallback

    /// When torch mode is changes, this callback will be called
    let torchModeChangeCallback: TorchModeChangeCallback

    /// When zoom scale is changes, this callback will be called
    let zoomScaleChangeCallback: ZoomScaleChangeCallback

    /// If provided, the Flutter registry will be used to send the output of the CaptureOutput to a Flutter texture.
    private let registry: FlutterTextureRegistry?

    /// Image to be sent to the texture
    var latestBuffer: CVImageBuffer!

    /// Texture id of the camera preview for Flutter
    private var textureId: Int64!

    var detectionSpeed: DetectionSpeed = DetectionSpeed.noDuplicates

    private let backgroundQueue = DispatchQueue(label: "camera-handling")

    var standardZoomFactor: CGFloat = 1

    private var nextScanTime = 0.0
    
    private var imagesCurrentlyBeingProcessed = false
    
    public var timeoutSeconds: Double = 0

    private var brightnessChangeCallback: ((Double) -> Void)?

    init(registry: FlutterTextureRegistry?, mobileScannerCallback: @escaping MobileScannerCallback, torchModeChangeCallback: @escaping TorchModeChangeCallback, zoomScaleChangeCallback: @escaping ZoomScaleChangeCallback, brightnessChangeCallback: @escaping (Double) -> Void) {
        self.registry = registry
        self.mobileScannerCallback = mobileScannerCallback
        self.torchModeChangeCallback = torchModeChangeCallback
        self.zoomScaleChangeCallback = zoomScaleChangeCallback
        self.brightnessChangeCallback = brightnessChangeCallback
        super.init()
    }

    /// Get the default camera device for the given `position`.
    ///
    /// This function selects the most appropriate camera, when it is available.
    private func getDefaultCameraDevice(position: AVCaptureDevice.Position) -> AVCaptureDevice? {
        if #available(iOS 13.0, *) {
            // Find the built-in Triple Camera, if it exists.
            if let device = AVCaptureDevice.default(.builtInTripleCamera,
                                                    for: .video,
                                                    position: position) {
                return device
            }
            
            // Find the built-in Dual-Wide Camera, if it exists.
            if let device = AVCaptureDevice.default(.builtInDualWideCamera,
                                                    for: .video,
                                                    position: position) {
                return device
            }
        }
        
        // Find the built-in Dual Camera, if it exists.
        if let device = AVCaptureDevice.default(.builtInDualCamera,
                                                for: .video,
                                                position: position) {
            return device
        }
        
        // Find the built-in Wide-Angle Camera, if it exists.
        if let device = AVCaptureDevice.default(.builtInWideAngleCamera,
                                                for: .video,
                                                position: position) {
            return device
        }
        
        return nil
    }
    
    /// Check if we already have camera permission.
    func checkPermission() -> Int {
        let status = AVCaptureDevice.authorizationStatus(for: .video)
        switch status {
        case .notDetermined:
            return 0
        case .authorized:
            return 1
        default:
            return 2
        }
    }

    /// Request permissions for video
    func requestPermission(_ result: @escaping FlutterResult) {
        AVCaptureDevice.requestAccess(for: .video, completionHandler: { result($0) })
    }
    
    /// Gets called when a new image is added to the buffer
    public func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        guard let imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else {
            return
        }
        latestBuffer = imageBuffer
        registry?.textureFrameAvailable(textureId)
        
        let currentTime = Date().timeIntervalSince1970
        let eligibleForScan = currentTime > nextScanTime && !imagesCurrentlyBeingProcessed
        
        if ((detectionSpeed == DetectionSpeed.normal || detectionSpeed == DetectionSpeed.noDuplicates) && eligibleForScan || detectionSpeed == DetectionSpeed.unrestricted) {

            nextScanTime = currentTime + timeoutSeconds
            imagesCurrentlyBeingProcessed = true
            
            let ciImage = latestBuffer.image

            let image = VisionImage(image: ciImage)
            image.orientation = imageOrientation(
                deviceOrientation: UIDevice.current.orientation,
                defaultOrientation: .portrait,
                position: videoPosition
            )

            scanner?.process(image) { [self] barcodes, error in
                imagesCurrentlyBeingProcessed = false
                
                if (detectionSpeed == DetectionSpeed.noDuplicates) {
                    let newScannedBarcodes = barcodes?.compactMap({ barcode in
                        return barcode.rawValue
                    }).sorted()
                    
                    if (error == nil && barcodesString != nil && newScannedBarcodes != nil && barcodesString!.elementsEqual(newScannedBarcodes!)) {
                        return
                    }
                    
                    if (newScannedBarcodes?.isEmpty == false) {
                        barcodesString = newScannedBarcodes
                    }
                }

                mobileScannerCallback(barcodes, error, ciImage)
            }
        }

        // Calculate brightness
        if let imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) {
            let brightness = calculateBrightness(from: imageBuffer)
            DispatchQueue.main.async {
                self.brightnessChangeCallback?(brightness)
            }
        }
    }

    private func calculateBrightness(from imageBuffer: CVImageBuffer) -> Double {
        CVPixelBufferLockBaseAddress(imageBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(imageBuffer, .readOnly) }

        let width = CVPixelBufferGetWidth(imageBuffer)
        let height = CVPixelBufferGetHeight(imageBuffer)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer)
        let baseAddress = CVPixelBufferGetBaseAddress(imageBuffer)

        var totalBrightness: Double = 0
        for y in 0..<height {
            let rowStart = baseAddress!.advanced(by: y * bytesPerRow)
            for x in 0..<width {
                let pixel = rowStart.advanced(by: x * 4)
                let r = Double(pixel.load(as: UInt8.self))
                let g = Double(pixel.load(fromByteOffset: 1, as: UInt8.self))
                let b = Double(pixel.load(fromByteOffset: 2, as: UInt8.self))
                totalBrightness += (0.299 * r + 0.587 * g + 0.114 * b) / 255.0
            }
        }

        return totalBrightness / Double(width * height)
    }

    /// Start scanning for barcodes
    func start(barcodeScannerOptions: BarcodeScannerOptions?, cameraPosition: AVCaptureDevice.Position, torch: Bool, detectionSpeed: DetectionSpeed, completion: @escaping (MobileScannerStartParameters) -> ()) throws {
        self.detectionSpeed = detectionSpeed
        if (device != nil || captureSession != nil) {
            throw MobileScannerError.alreadyStarted
        }

        barcodesString = nil
        scanner = barcodeScannerOptions != nil ? BarcodeScanner.barcodeScanner(options: barcodeScannerOptions!) : BarcodeScanner.barcodeScanner()
        captureSession = AVCaptureSession()
        textureId = registry?.register(self)

        // Open the camera device
        device = getDefaultCameraDevice(position: cameraPosition)

        if (device == nil) {
            throw MobileScannerError.noCamera
        }

        device.addObserver(self, forKeyPath: #keyPath(AVCaptureDevice.torchMode), options: .new, context: nil)
        device.addObserver(self, forKeyPath: #keyPath(AVCaptureDevice.videoZoomFactor), options: .new, context: nil)

        // Check the zoom factor at switching from ultra wide camera to wide camera.
        standardZoomFactor = 1
        if #available(iOS 13.0, *) {
            for (index, actualDevice) in device.constituentDevices.enumerated() {
                if (actualDevice.deviceType != .builtInUltraWideCamera) {
                    if index > 0 && index <= device.virtualDeviceSwitchOverVideoZoomFactors.count {
                        standardZoomFactor = CGFloat(truncating: device.virtualDeviceSwitchOverVideoZoomFactors[index - 1])
                    }
                    break
                }
            }
        }

        do {
            try device.lockForConfiguration()
            if device.isFocusModeSupported(.continuousAutoFocus) {
                device.focusMode = .continuousAutoFocus
            }
            if #available(iOS 15.4, *) , device.isFocusModeSupported(.autoFocus){
                device.automaticallyAdjustsFaceDrivenAutoFocusEnabled = false
            }
            device.unlockForConfiguration()
        } catch {}

        captureSession!.beginConfiguration()

        // Add device input
        do {
            let input = try AVCaptureDeviceInput(device: device)
            captureSession!.addInput(input)
        } catch {
            throw MobileScannerError.cameraError(error)
        }

        captureSession!.sessionPreset = AVCaptureSession.Preset.photo
        // Add video output.
        let videoOutput = AVCaptureVideoDataOutput()

        videoOutput.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA]
        videoOutput.alwaysDiscardsLateVideoFrames = true

        videoPosition = cameraPosition
        // calls captureOutput()
        videoOutput.setSampleBufferDelegate(self, queue: DispatchQueue.main)

        captureSession!.addOutput(videoOutput)
        for connection in videoOutput.connections {
            connection.videoOrientation = .portrait
            if cameraPosition == .front && connection.isVideoMirroringSupported {
                connection.isVideoMirrored = true
            }
        }
        captureSession!.commitConfiguration()

        backgroundQueue.async {
            guard let captureSession = self.captureSession else {
                return
            }

            captureSession.startRunning()

            // After the capture session started, turn on the torch (if requested)
            // and reset the zoom scale back to the default.
            // Ensure that these adjustments are done on the main DispatchQueue,
            // as they interact with the hardware camera.
            if (torch) {
                DispatchQueue.main.async {
                    self.turnTorchOn()
                }
            }
            
            DispatchQueue.main.async {
                do {
                    try self.resetScale()
                } catch {
                    // If the zoom scale could not be reset,
                    // continue with the capture session anyway.
                }
            }

            if let device = self.device {
                // When querying the dimensions of the camera,
                // stay on the background thread,
                // as this does not change the configuration of the hardware camera.
                let dimensions = CMVideoFormatDescriptionGetDimensions(
                    device.activeFormat.formatDescription)
                
                completion(
                    MobileScannerStartParameters(
                        width: Double(dimensions.height),
                        height: Double(dimensions.width),
                        currentTorchState: device.hasTorch ? device.torchMode.rawValue : -1,
                        textureId: self.textureId ?? 0
                    )
                )
                
                return
            }
            
            completion(MobileScannerStartParameters())
        }
    }

    /// Stop scanning for barcodes
    func stop() throws {
        if (device == nil || captureSession == nil) {
            throw MobileScannerError.alreadyStopped
        }
        
        captureSession!.stopRunning()
        for input in captureSession!.inputs {
            captureSession!.removeInput(input)
        }
        for output in captureSession!.outputs {
            captureSession!.removeOutput(output)
        }

        latestBuffer = nil
        device.removeObserver(self, forKeyPath: #keyPath(AVCaptureDevice.torchMode))
        device.removeObserver(self, forKeyPath: #keyPath(AVCaptureDevice.videoZoomFactor))
        registry?.unregisterTexture(textureId)
        textureId = nil
        captureSession = nil
        device = nil
        scanner = nil
    }

    /// Toggle the torch.
    ///
    /// This method should be called on the main DispatchQueue.
    func toggleTorch() {
        guard let device = self.device else {
            return
        }
        
        if (!device.hasTorch || !device.isTorchAvailable) {
            return
        }
        
        var newTorchMode: AVCaptureDevice.TorchMode = device.torchMode
        
        switch(device.torchMode) {
        case AVCaptureDevice.TorchMode.auto:
            newTorchMode = device.isTorchActive ? AVCaptureDevice.TorchMode.off : AVCaptureDevice.TorchMode.on
            break;
        case AVCaptureDevice.TorchMode.off:
            newTorchMode = AVCaptureDevice.TorchMode.on
            break;
        case AVCaptureDevice.TorchMode.on:
            newTorchMode = AVCaptureDevice.TorchMode.off
            break;
        default:
            return;
        }
        
        if (!device.isTorchModeSupported(newTorchMode) || device.torchMode == newTorchMode) {
            return;
        }

        do {
            try device.lockForConfiguration()
            device.torchMode = newTorchMode
            device.unlockForConfiguration()
        } catch(_) {}
    }
    
    /// Turn the torch on.
    private func turnTorchOn() {
        guard let device = self.device else {
            return
        }
        
        if (!device.hasTorch || !device.isTorchAvailable || !device.isTorchModeSupported(.on) || device.torchMode == .on) {
            return
        }
        
        do {
            try device.lockForConfiguration()
            device.torchMode = .on
            device.unlockForConfiguration()
        } catch(_) {}
    }

    // Observer for torch state
    public override func observeValue(forKeyPath keyPath: String?, of object: Any?, change: [NSKeyValueChangeKey : Any]?, context: UnsafeMutableRawPointer?) {
        switch keyPath {
        case "torchMode":
            // Off = 0, On = 1, Auto = 2
            let state = change?[.newKey] as? Int
            torchModeChangeCallback(state)
        case "videoZoomFactor":
            let zoomFactor = change?[.newKey] as? CGFloat ?? 1
            let zoomScale = (zoomFactor - 1) / 4
            zoomScaleChangeCallback(Double(zoomScale))
        default:
            break
        }
    }
    
    /// Set the zoom factor of the camera
    func setScale(_ scale: CGFloat) throws {
        if (device == nil) {
            throw MobileScannerError.zoomWhenStopped
        }
        
        do {
            try device.lockForConfiguration()
            let maxZoomFactor = device.activeFormat.videoMaxZoomFactor
            
            var actualScale = (scale * 4) + 1
            
            // Set maximum zoomrate of 5x
            actualScale = min(5.0, actualScale)
            
            // Limit to max rate of camera
            actualScale = min(maxZoomFactor, actualScale)
            
            // Limit to 1.0 scale
            device.videoZoomFactor = actualScale
            device.unlockForConfiguration()
        } catch {
            throw MobileScannerError.zoomError(error)
        }
        
    }

    /// Reset the zoom factor of the camera
    func resetScale() throws {
        if (device == nil) {
            throw MobileScannerError.zoomWhenStopped
        }

        do {
            try device.lockForConfiguration()
            device.videoZoomFactor = standardZoomFactor
            device.unlockForConfiguration()
        } catch {
            throw MobileScannerError.zoomError(error)
        }
    }

    func preProcessImage(_ image: UIImage) -> UIImage {
        // 转换为灰度图像
        var processedImage = image.convertToGrayscale()
        // 增强对比度
        processedImage = processedImage.enhanceContrast()
        // 应用自适应阈值
        processedImage = processedImage.applyAdaptiveThreshold()
        // 去噪
        processedImage = processedImage.denoise()
        // 锐化
        processedImage = processedImage.sharpen()
        return processedImage
    }

    /// Analyze a single image
    func analyzeImage(image: UIImage, position: AVCaptureDevice.Position,
                      barcodeScannerOptions: BarcodeScannerOptions?, callback: @escaping BarcodeScanningCallback) {
        // 创建扫描器
        let scanner: BarcodeScanner = barcodeScannerOptions != nil ? BarcodeScanner.barcodeScanner(options: barcodeScannerOptions!) : BarcodeScanner.barcodeScanner()
        
        // 定义三种不同的图像处理方式
        let imageProcessingSteps: [(UIImage) -> UIImage] = [
            { $0 }, // 原始图像
            { self.preProcessImage($0) }, // 预处理图像
            { self.preProcessImage($0).resize(to: CGSize(width: 1920, height: 1920)) } // 预处理并调整分辨率
        ]
        
        func attemptScan(with processedImage: UIImage, stepIndex: Int) {
            let visionImage = VisionImage(image: processedImage)
            visionImage.orientation = imageOrientation(
                deviceOrientation: UIDevice.current.orientation,
                defaultOrientation: .portrait,
                position: position
            )
            
            scanner.process(visionImage) { barcodes, error in
                if let barcodes = barcodes, !barcodes.isEmpty {
                    // 成功识别
                    callback(barcodes, nil)
                } else if stepIndex < imageProcessingSteps.count - 1 {
                    // 尝试下一种图像处理方式
                    let nextProcessedImage = imageProcessingSteps[stepIndex + 1](image)
                    attemptScan(with: nextProcessedImage, stepIndex: stepIndex + 1)
                } else {
                    // 所有尝试都失败
                    callback(nil, error)
                }
            }
        }
        
        // 开始第一次尝试，使用原始图像
        attemptScan(with: image, stepIndex: 0)
    }

    var barcodesString: Array<String?>?

    /// Rotates images accordingly
    func imageOrientation(
        deviceOrientation: UIDeviceOrientation,
        defaultOrientation: UIDeviceOrientation,
        position: AVCaptureDevice.Position
    ) -> UIImage.Orientation {
        switch deviceOrientation {
        case .portrait:
            return position == .front ? .leftMirrored : .right
        case .landscapeLeft:
            return position == .front ? .downMirrored : .up
        case .portraitUpsideDown:
            return position == .front ? .rightMirrored : .left
        case .landscapeRight:
            return position == .front ? .upMirrored : .down
        case .faceDown, .faceUp, .unknown:
            return .up
        @unknown default:
            return imageOrientation(deviceOrientation: defaultOrientation, defaultOrientation: .portrait, position: .back)
        }
    }

    /// Sends output of OutputBuffer to a Flutter texture
    public func copyPixelBuffer() -> Unmanaged<CVPixelBuffer>? {
        if latestBuffer == nil {
            return nil
        }
        return Unmanaged<CVPixelBuffer>.passRetained(latestBuffer)
    }
    
    struct MobileScannerStartParameters {
        var width: Double = 0.0
        var height: Double = 0.0
        var currentTorchState: Int = -1
        var textureId: Int64 = 0
    }
}

extension UIImage {
    func convertToGrayscale() -> UIImage {
        let context = CIContext(options: nil)
        guard let currentFilter = CIFilter(name: "CIPhotoEffectMono") else { return self }
        currentFilter.setValue(CIImage(image: self), forKey: kCIInputImageKey)
        if let output = currentFilter.outputImage,
           let cgImage = context.createCGImage(output, from: output.extent) {
            return UIImage(cgImage: cgImage)
        }
        return self
    }
    
    func enhanceContrast() -> UIImage {
        guard let currentFilter = CIFilter(name: "CIColorControls") else { return self }
        currentFilter.setValue(CIImage(image: self), forKey: kCIInputImageKey)
        currentFilter.setValue(1.1, forKey: kCIInputContrastKey)
        currentFilter.setValue(0.1, forKey: kCIInputBrightnessKey)
        if let output = currentFilter.outputImage,
           let cgImage = CIContext(options: nil).createCGImage(output, from: output.extent) {
            return UIImage(cgImage: cgImage)
        }
        return self
    }
    
    func applyAdaptiveThreshold() -> UIImage {
        // 这里使用 Core Image 的 CIColorThreshold 滤镜作为示例
        // 实际应用中可能需要更复杂的自适应阈值算法
        guard let currentFilter = CIFilter(name: "CIColorThreshold") else { return self }
        currentFilter.setValue(CIImage(image: self), forKey: kCIInputImageKey)
        currentFilter.setValue(0.5, forKey: "inputThreshold")
        if let output = currentFilter.outputImage,
           let cgImage = CIContext(options: nil).createCGImage(output, from: output.extent) {
            return UIImage(cgImage: cgImage)
        }
        return self
    }
    
    func denoise() -> UIImage {
        guard let currentFilter = CIFilter(name: "CINoiseReduction") else { return self }
        currentFilter.setValue(CIImage(image: self), forKey: kCIInputImageKey)
        currentFilter.setValue(0.02, forKey: "inputNoiseLevel")
        currentFilter.setValue(0.40, forKey: "inputSharpness")
        if let output = currentFilter.outputImage,
           let cgImage = CIContext(options: nil).createCGImage(output, from: output.extent) {
            return UIImage(cgImage: cgImage)
        }
        return self
    }
    
    func sharpen() -> UIImage {
        guard let currentFilter = CIFilter(name: "CISharpenLuminance") else { return self }
        currentFilter.setValue(CIImage(image: self), forKey: kCIInputImageKey)
        currentFilter.setValue(0.5, forKey: "inputSharpness")
        if let output = currentFilter.outputImage,
           let cgImage = CIContext(options: nil).createCGImage(output, from: output.extent) {
            return UIImage(cgImage: cgImage)
        }
        return self
    }

    func resize(to targetSize: CGSize) -> UIImage {
        let widthRatio  = targetSize.width  / size.width
        let heightRatio = targetSize.height / size.height
        
        // 使用较小的比率来确保图像适应目标大小
        let scaleFactor = min(widthRatio, heightRatio)
        
        let scaledWidth  = size.width * scaleFactor
        let scaledHeight = size.height * scaleFactor
        let targetRect = CGRect(
            x: (targetSize.width - scaledWidth) / 2,
            y: (targetSize.height - scaledHeight) / 2,
            width: scaledWidth,
            height: scaledHeight
        )
        
        UIGraphicsBeginImageContextWithOptions(targetSize, false, 0)
        draw(in: targetRect)
        let newImage = UIGraphicsGetImageFromCurrentImageContext()
        UIGraphicsEndImageContext()
        
        return newImage ?? self
    }
}
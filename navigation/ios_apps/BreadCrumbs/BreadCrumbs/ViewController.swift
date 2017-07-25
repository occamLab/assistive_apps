//
//  ViewController.swift
//  ARKitTest
//
//  Created by Chris Seonghwan Yoon & Jeremy Ryan on 7/10/17.
//

import UIKit
import ARKit
import SceneKit
import SceneKit.ModelIO
import AVFoundation

extension UIView {
    func fadeTransition(_ duration:CFTimeInterval) {
        let animation = CATransition()
        animation.timingFunction = CAMediaTimingFunction(name:
            kCAMediaTimingFunctionEaseInEaseOut)
        animation.type = kCATransitionPush
        animation.subtype = kCATransitionFromTop
        animation.duration = duration
        layer.add(animation, forKey: kCATransitionFade)
    }
}

class ViewController: UIViewController {
    
    /*                      UI Configuration                   */
    /*---------------------------------------------------------*/
    
    // hide status bar
    override var prefersStatusBarHidden: Bool {
        return true
    }
    
    // button frame extends the entire width of screen
    var buttonFrameWidth: CGFloat {
        return UIScreen.main.bounds.size.width
    }
    
    // height of button frame
    var buttonFrameHeight: CGFloat {
        return UIScreen.main.bounds.size.height * (1/5)
    }
    
    var displayWidth: CGFloat {
        return UIScreen.main.bounds.size.width
    }
    
    var displayHeight: CGFloat {
        return UIScreen.main.bounds.size.height
    }
    
    // top margin of direction text label
    var textLabelBuffer: CGFloat {
        return buttonFrameHeight * (1/12)
    }
    
    // y-origin of button frame
    var yOriginOfButtonFrame: CGFloat {
        return UIScreen.main.bounds.size.height - buttonFrameHeight
    }
    
    /* UIViewss for all UI button containers */
    var getDirectionButton: UIButton!
    var recordPathView: UIView!
    var stopRecordingView: UIView!
    var startNavigationView: UIView!
    var stopNavigationView: UIView!
    var directionText: UILabel!
    var routeRatingView: UIView!
    
    // State of button views
    enum ButtonViewType {
        case recordPath
        case stopRecording
        case startNavigation
        case stopNavigation
    }
    
    // current button in display
    var currentButton = ButtonViewType.recordPath
    
    override func viewDidLoad() {
        super.viewDidLoad()
        createSettingsBundle()
        createARSession()
        drawUI()
        addGestures()
    }
    
    func createSettingsBundle() {
        registerSettingsBundle()
        updateDisplayFromDefaults()
        NotificationCenter.default.addObserver(self,
                                               selector: #selector(defaultsChanged),
                                               name: UserDefaults.didChangeNotification,
                                               object: nil)
    }
    
    func registerSettingsBundle(){
        let appDefaults = [String:AnyObject]()
        UserDefaults.standard.register(defaults: appDefaults)
    }
    
    func updateDisplayFromDefaults(){
        let defaults = UserDefaults.standard
        defaultUnit = defaults.integer(forKey: "units")
        defaultColor = defaults.integer(forKey: "crumbColor")
    }
    
    @objc func defaultsChanged(){
        updateDisplayFromDefaults()
    }
    
    /*
     * Creates a new ARSession with ARWorldTracking session config
     */
    func createARSession() {
        let configuration = ARWorldTrackingSessionConfiguration()
        configuration.planeDetection = .horizontal
        sceneView.session.run(configuration)
        sceneView.backgroundColor = UIColor(patternImage: UIImage(named: "SplashScreen")!)
    }
    
    /*
     * Adds TapGesture to the sceneView
     */
    func addGestures() {
        let tapGestureRecognizer = UITapGestureRecognizer(target: self, action: #selector(announceDirectionHelp))
        tapGestureRecognizer.numberOfTapsRequired = 2
        self.view.addGestureRecognizer(tapGestureRecognizer)
    }
    
    func drawUI() {
        // button that gives direction to the nearist keypoint
        getDirectionButton = UIButton(frame: CGRect(x: 0, y: 0, width: buttonFrameWidth, height: yOriginOfButtonFrame))
        getDirectionButton.isAccessibilityElement = true
        getDirectionButton.accessibilityLabel = "Get Directions"
        getDirectionButton.isHidden = true
        getDirectionButton.addTarget(self, action: #selector(aannounceDirectionHelpPressed), for: .touchUpInside)
        
        // textlabel that displys directions
        directionText = UILabel(frame: CGRect(x: 0, y: (yOriginOfButtonFrame + textLabelBuffer), width: buttonFrameWidth, height: buttonFrameHeight*(1/6)))
        directionText.textColor = UIColor.white
        directionText.textAlignment = .center
        directionText.isAccessibilityElement = false
        
        // Record Path button container
        recordPathView = UIView(frame: CGRect(x: 0, y: yOriginOfButtonFrame, width: buttonFrameWidth, height: buttonFrameHeight))
        recordPathView.backgroundColor = UIColor.black.withAlphaComponent(0.4)
        recordPathView.isHidden = false
        addButtons(buttonView: recordPathView, buttonViewType: .recordPath)
        
        // Stop Recording button container
        stopRecordingView = UIView(frame: CGRect(x: 0, y: yOriginOfButtonFrame, width: buttonFrameWidth, height: buttonFrameHeight))
        stopRecordingView.backgroundColor = UIColor.black.withAlphaComponent(0.4)
        stopRecordingView.isHidden = true
        addButtons(buttonView: stopRecordingView, buttonViewType: .stopRecording)
        
        // Start Navigation button container
        startNavigationView = UIView(frame: CGRect(x: 0, y: yOriginOfButtonFrame, width: buttonFrameWidth, height: buttonFrameHeight))
        startNavigationView.backgroundColor = UIColor.black.withAlphaComponent(0.4)
        startNavigationView.isHidden = true
        addButtons(buttonView: startNavigationView, buttonViewType: .startNavigation)
        
        // Stop Navigation button container
        stopNavigationView = UIView(frame: CGRect(x: 0, y: yOriginOfButtonFrame, width: buttonFrameWidth, height: buttonFrameHeight))
        stopNavigationView.backgroundColor = UIColor.black.withAlphaComponent(0.4)
        stopNavigationView.isHidden = true
        addButtons(buttonView: stopNavigationView, buttonViewType: .stopNavigation)
        
        routeRatingView = UIView(frame: CGRect(x: 0, y: 0, width: UIScreen.main.bounds.size.width, height: UIScreen.main.bounds.size.height))
        routeRatingView.backgroundColor = UIColor.black.withAlphaComponent(0.4)
        drawRouteRatingView()
        
        self.view.addSubview(recordPathView)
        self.view.addSubview(stopRecordingView)
        self.view.addSubview(startNavigationView)
        self.view.addSubview(stopNavigationView)
        self.view.addSubview(directionText)
        self.view.addSubview(getDirectionButton)
        self.view.addSubview(routeRatingView)
        showRecordPathButton()
    }
    
    func drawRouteRatingView() {
        let label = UILabel(frame: CGRect(x: 0, y: displayHeight/2.5, width: displayWidth, height: displayHeight/6))
        label.text = "Please rate your navigation service."
        label.textColor = UIColor.white
        label.textAlignment = .center
        
        let buttonWidth = routeRatingView.bounds.size.width / 4.5
        
        let thumbsUpButton = UIButton(type: .custom)
        thumbsUpButton.frame = CGRect(x: 0, y: 0, width: buttonWidth , height: buttonWidth)
        thumbsUpButton.layer.cornerRadius = 0.5 * thumbsUpButton.bounds.size.width
        thumbsUpButton.clipsToBounds = true
        thumbsUpButton.setTitle("Good", for: .normal)
        thumbsUpButton.layer.borderWidth = 2
        thumbsUpButton.layer.borderColor = UIColor.white.cgColor
        thumbsUpButton.center.x = routeRatingView.center.x + displayWidth/4
        thumbsUpButton.center.y = routeRatingView.bounds.size.height * (2/3)
       thumbsUpButton.addTarget(self, action: #selector(sendLogData), for: .touchUpInside)
        
        let thumbsDownButton = UIButton(type: .custom)
        thumbsDownButton.frame = CGRect(x: 0, y: 0, width: buttonWidth , height: buttonWidth)
        thumbsDownButton.layer.cornerRadius = 0.5 * thumbsUpButton.bounds.size.width
        thumbsDownButton.clipsToBounds = true
        thumbsDownButton.setTitle("Bad", for: .normal)
        thumbsDownButton.layer.borderWidth = 2
        thumbsDownButton.layer.borderColor = UIColor.white.cgColor
        thumbsDownButton.center.x = routeRatingView.center.x - displayWidth/4
        thumbsDownButton.center.y = routeRatingView.bounds.size.height * (2/3)
        thumbsDownButton.addTarget(self, action: #selector(sendDebugLogData), for: .touchUpInside)
        
        routeRatingView.addSubview(thumbsDownButton)
        routeRatingView.addSubview(thumbsUpButton)
        routeRatingView.addSubview(label)
    }
    
    /*
     * Adds buttons to given UIView container
     */
    func addButtons(buttonView: UIView, buttonViewType: ButtonViewType) {
        let buttonWidth = buttonView.bounds.size.width / 4.5
        
        let button = UIButton(type: .custom)
        button.frame = CGRect(x: 0, y: 0, width: buttonWidth , height: buttonWidth )
        button.layer.cornerRadius = 0.5 * button.bounds.size.width
        button.clipsToBounds = true
        
        button.center.x = buttonView.center.x
        button.center.y = buttonView.bounds.size.height * (6/10)
        
        // Adds custom button labels
        switch buttonViewType {
        case .recordPath:
            let buttonImage = UIImage(named: "StartRecording")
            button.setImage(buttonImage, for: .normal)
            button.accessibilityLabel = "Reecord Path"
            button.addTarget(self, action: #selector(recordPath), for: .touchUpInside)
        case.stopRecording:
            let buttonImage = UIImage(named: "StopRecording")
            button.setImage(buttonImage, for: .normal)
            button.accessibilityLabel = "Stop recording"
            button.addTarget(self, action: #selector(stopRecording), for: .touchUpInside)
        case .startNavigation:
            let buttonImage = UIImage(named: "StartNavigation")
            button.setImage(buttonImage, for: .normal)
            button.accessibilityLabel = "Start Navigation"
            button.addTarget(self, action: #selector(startNavigation), for: .touchUpInside)
        case.stopNavigation:
            let buttonImage = UIImage(named: "StopNavigation")
            button.setImage(buttonImage, for: .normal)
            button.accessibilityLabel = "Stop Navigation"
            button.addTarget(self, action: #selector(stopNavigation), for: .touchUpInside)
        }
        
        buttonView.addSubview(button)
    }
    
    /*
     * display RECORD PATH button/hide all other views
     */
    @objc func showRecordPathButton() {
        recordPathView.isHidden = false
        stopNavigationView.isHidden = true
        getDirectionButton.isHidden = true
        directionText.isHidden = false
        routeRatingView.isHidden = true
        navigationMode = false
        currentButton = .recordPath
        updateDirectionText("Press to record path", distance: 0, size: 16, displayDistance: false)
    }
    
    /*
     * display STOP RECORDIN button/hide all other views
     */
    @objc func showStopRecordingButton() {
        recordPathView.isHidden = true
        stopRecordingView.isHidden = false
        navigationMode = false
        currentButton = .stopRecording
        updateDirectionText("Recording Path", distance: 0, size: 16, displayDistance: false)
    }
    
    /*
     * display START NAVIGATION button/hide all other views
     */
    @objc func showStartNavigationButton() {
        stopRecordingView.isHidden = true
        startNavigationView.isHidden = false
        navigationMode = false
        currentButton = .startNavigation
        updateDirectionText("Press to start navigation", distance: 0, size: 16, displayDistance: false)
    }
    
    /*
     * display STOP NAVIGATION button/hide all other views
     */
    @objc func showStopNavigationButton() {
        startNavigationView.isHidden = true
        stopNavigationView.isHidden = false
        getDirectionButton.isHidden = false
        currentButton = .stopNavigation
    }
    
    @objc func showRouteRating() {
        stopNavigationView.isHidden = true
        getDirectionButton.isHidden = true
        directionText.isHidden = true
        routeRatingView.isHidden = false
        currentButton = .stopNavigation
    }
    
    /*
     * update directionText UILabel given text string and font size
     * distance Bool used to determine whether to add string "meters" to direction text
     */
    func updateDirectionText(_ discription: String, distance: Float, size: CGFloat, displayDistance: Bool) {
        directionText.fadeTransition(0.4)
        directionText.font = directionText.font.withSize(size)
        var altText = ""
        if(displayDistance) {
            directionText.text = discription + " for \(distance)" + unit[defaultUnit]!
            if(defaultUnit == 0) {
                altText = discription + " for \(Int(distance))" + unitText[defaultUnit]!
            } else {
                if(distance >= 10) {
                    let integer = Int(distance)
                    let decimal = Int((distance - Float(integer)) * 10)
                    altText = discription + "\(integer) point \(decimal)" + unitText[defaultUnit]!
                } else {
                    altText = discription + "\(distance)" + unitText[defaultUnit]!
                }
            }
        } else {
            directionText.text = discription
            altText = discription
        }
        if(navigationMode) {
            speechData.append([altText, -dataTimer.timeIntervalSinceNow])
            getRealCoordinates(sceneView: sceneView)
        }
        UIAccessibilityPostNotification(UIAccessibilityAnnouncementNotification, altText)
    }
    
    /*                  BreadCrumbs main logic                 */
    /*---------------------------------------------------------*/
    
    var crumbs: [LocationInfo]!         // List of crumbs dropped when recording path
    var keypoints: [KeypointInfo]!      // List of keypoints calculated after path completion
    var keypointNode: SCNNode!          // SCNNode of the next keypoint
    
    var dataTimer: Date!
    var pathData: [Array<Any>]!
    var pathDataTime: [Double]!
    var navigationData: [Array<Any>]!
    var navigationDataTime: [Double]!
    var speechData: [Array<Any>]!
    
    /* Timers for background functions */
    var droppingCrumbs: Timer!
    var followingCrumbs: Timer!
    var announcementTimer: Timer!
    
    var nav = Navigation()                  // Navigation calculation class
    var navigationMode: Bool = false        // navigation flag
    
    public let unit = [0: "ft", 1: "m"]
    public let unitText = [0: " feet", 1: " meters"]
    public var defaultUnit: Int!
    public var defaultColor: Int!
    
    @IBOutlet weak var sceneView: ARSCNView!
    
    @objc func recordPath() {
        crumbs = []
        pathData = []
        pathDataTime = []
        dataTimer = Date()
        announcementTimer = Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: (#selector(showStopRecordingButton)), userInfo: nil, repeats: false)
        droppingCrumbs = Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: #selector(dropCrum), userInfo: nil, repeats: true)
    }
    
    @objc func stopRecording(_ sender: UIButton) {
        droppingCrumbs.invalidate()
        announcementTimer = Timer.scheduledTimer(timeInterval: 1, target: self, selector: (#selector(showStartNavigationButton)), userInfo: nil, repeats: false)
    }
    
    @objc func startNavigation(_ sender: UIButton) {
        navigationData = []
        navigationDataTime = []
        speechData = []
        dataTimer = Date()
        
        let path = PathFinder(crums: crumbs.reversed())
        keypoints = path.keypoints
        
        renderKeypoint(keypoints[0].location)
        navigationMode = true
        announcementTimer = Timer.scheduledTimer(timeInterval: 1, target: self, selector: (#selector(showStopNavigationButton)), userInfo: nil, repeats: false)
        followingCrumbs = Timer.scheduledTimer(timeInterval: 0.2, target: self, selector: (#selector(followCrum)), userInfo: nil, repeats: true)
    }
    
    @objc func stopNavigation(_ sender: UIButton) {
        followingCrumbs.invalidate()
        announcementTimer = Timer.scheduledTimer(timeInterval: 1, target: self, selector: (#selector(showRecordPathButton)), userInfo: nil, repeats: false)
        showRouteRating()
    }
    
    @objc func sendLogData() {
        compileLogData(false)
        showRecordPathButton()
    }
    
    @objc func sendDebugLogData() {
        compileLogData(true)
        showRecordPathButton()
    }
    
    func compileLogData(_ debug: Bool) {
        print("compling log..")
        let log: [String : Any] = ["pathData": pathData,
                                   "pathDataTime": pathDataTime,
                                   "navigationData": navigationData,
                                   "navigationDataTime": navigationDataTime,
                                   "speechData": speechData]
        
        do {
            let jsonData = try JSONSerialization.data(withJSONObject: log, options: .prettyPrinted)
            // here "jsonData" is the dictionary encoded in JSON data
            let string1 = String(data: jsonData, encoding: String.Encoding.utf8) ?? "Data could not be printed"
            print(string1)
        } catch {
            print(error.localizedDescription)
        }
    }
    
    @objc func dropCrum() {
        let curLocation = getRealCoordinates(sceneView: sceneView).location
        crumbs.append(curLocation)
    }
    
    @objc func followCrum() {
        let curLocation = getRealCoordinates(sceneView: sceneView)
        var directionToNextKeypoint = getDirectionToNextKeypoint(currentLocation: curLocation)
        
        if(directionToNextKeypoint.targetState == PositionState.atTarget) {
            if (keypoints.count > 1) {
                keypointNode.removeFromParentNode()
                keypoints.remove(at: 0)
                renderKeypoint(keypoints[0].location)
                directionToNextKeypoint = getDirectionToNextKeypoint(currentLocation: curLocation)
                setDirectionText(currentLocation: curLocation.location, direction: directionToNextKeypoint, displayDistance: false)
            } else {
                keypointNode.removeFromParentNode()
                announceArrival()
                followingCrumbs.invalidate()
            }
        }
        
    }
    
    func getDirectionToNextKeypoint(currentLocation: CurrentCoordinateInfo) -> DirectionInfo {
        var dir = nav.getDirections(currentLocation: currentLocation, nextKeypoint: keypoints[0])
        if(defaultUnit == 0) {
            dir.distance *= 3.28084
        }
        dir.distance = roundToTenths(dir.distance)
        return dir
    }
    
    func roundToTenths(_ n: Float) -> Float {
        return roundf(10 * n)/10
    }
    
    @objc func aannounceDirectionHelpPressed() {
        announcementTimer = Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: (#selector(announceDirectionHelp)), userInfo: nil, repeats: false)
    }
    
    /*
     * Announce directions at any given point to the next keypoint
     */
    @objc func announceDirectionHelp() {
        if (navigationMode) {
            let curLocation = getRealCoordinates(sceneView: sceneView)
            let directionToNextKeypoint = getDirectionToNextKeypoint(currentLocation: curLocation)
            setDirectionText(currentLocation: curLocation.location, direction: directionToNextKeypoint, displayDistance: true)
        }
    }
    
    /*
     * Set direction text for text label and VoiceOver
     */
    func setDirectionText(currentLocation: LocationInfo, direction: DirectionInfo, displayDistance: Bool) {
        let xzNorm = sqrtf(powf(currentLocation.x - keypoints[0].location.x, 2) + powf(currentLocation.z - keypoints[0].location.z, 2))
        let slope = (keypoints[0].location.y - currentLocation.y) / xzNorm
        var dir = ""
        
        if(slope > 0.3) { // Go upstairs
            dir = "\(Directions[direction.clockDirection]!) and proceed upstairs."
            updateDirectionText(dir, distance: 0, size: 12, displayDistance: false)
        } else if (slope < -0.3) { // Go downstairs
            dir = "\(Directions[direction.clockDirection]!) and proceed downstairs."
            updateDirectionText(dir, distance: direction.distance,size: 12, displayDistance: false)
        } else { // nromal directions
            dir = "\(Directions[direction.clockDirection]!)"
            updateDirectionText(dir, distance: direction.distance, size: 16, displayDistance:  displayDistance)
        }
    }
    
    func announceArrival() {
        updateDirectionText("You have arrived!", distance: 0, size: 16, displayDistance: false)
        announcementTimer = Timer.scheduledTimer(timeInterval: 1, target: self, selector: (#selector(showRouteRating)), userInfo: nil, repeats: false)
    }
    
    func renderKeypoint(_ location: LocationInfo) {
        let bundle = Bundle.main
        let path = bundle.path(forResource: "Crumb", ofType: "obj")
        let url = NSURL(fileURLWithPath: path!)
        let asset = MDLAsset(url: url as URL)
        let object = asset.object(at: 0)
        keypointNode = SCNNode(mdlObject: object)
        
        keypointNode.scale = SCNVector3(0.0004, 0.0004, 0.0004)
        keypointNode.geometry?.firstMaterial?.diffuse.contents = UIColor.red
        keypointNode.position = SCNVector3(location.x, location.y - 0.2, location.z)
        keypointNode.rotation = SCNVector4(0, 1, 0, (location.yaw - Float.pi/2))
        
        let bound = SCNVector3(
                x: keypointNode.boundingBox.max.x - keypointNode.boundingBox.min.x,
                y: keypointNode.boundingBox.max.y - keypointNode.boundingBox.min.y,
                z: keypointNode.boundingBox.max.z - keypointNode.boundingBox.min.z)
            
        keypointNode.pivot = SCNMatrix4MakeTranslation(bound.x / 2, bound.y / 2, bound.z / 2)
        
        let spin = CABasicAnimation(keyPath: "rotation")
        spin.fromValue = NSValue(scnVector4: SCNVector4(x: 0, y: 1, z: 0, w: 0))
        spin.toValue = NSValue(scnVector4: SCNVector4(x: 0, y: 1, z: 0, w: Float(CGFloat(2 * Float.pi))))
        spin.duration = 3
        spin.repeatCount = .infinity
        keypointNode.addAnimation(spin, forKey: "spin around")
        
        let flashRed = SCNAction.customAction(duration: 2) { (node, elapsedTime) -> () in
            let percentage = Float(elapsedTime / 2)
            var color = UIColor.clear
            let power: Float = 2.0
            
            
            if (percentage < 0.5) {
                color = UIColor(red: 1,
                                green: CGFloat(powf(2.0*percentage, power)),
                                blue: CGFloat(powf(2.0*percentage, power)),
                                alpha: 1)
            } else {
                color = UIColor(red: 1,
                                green: CGFloat(powf(2-2.0*percentage, power)),
                                blue: CGFloat(powf(2-2.0*percentage, power)),
                                alpha: 1)
            }
            node.geometry!.firstMaterial!.diffuse.contents = color
        }
        
        let flashGreen = SCNAction.customAction(duration: 2) { (node, elapsedTime) -> () in
            let percentage = Float(elapsedTime / 2)
            var color = UIColor.clear
            let power: Float = 2.0
            
            
            if (percentage < 0.5) {
                color = UIColor(red: CGFloat(powf(2.0*percentage, power)),
                                green: 1,
                                blue: CGFloat(powf(2.0*percentage, power)),
                                alpha: 1)
            } else {
                color = UIColor(red: CGFloat(powf(2-2.0*percentage, power)),
                                green: 1,
                                blue: CGFloat(powf(2-2.0*percentage, power)),
                                alpha: 1)
            }
            node.geometry!.firstMaterial!.diffuse.contents = color
        }
        
        let flashBlue = SCNAction.customAction(duration: 2) { (node, elapsedTime) -> () in
            let percentage = Float(elapsedTime / 2)
            var color = UIColor.clear
            let power: Float = 2.0
            
            
            if (percentage < 0.5) {
                color = UIColor(red: CGFloat(powf(2.0*percentage, power)),
                                green: CGFloat(powf(2.0*percentage, power)),
                                blue: 1,
                                alpha: 1)
            } else {
                color = UIColor(red: CGFloat(powf(2-2.0*percentage, power)),
                                green: CGFloat(powf(2-2.0*percentage, power)),
                                blue: 1,
                                alpha: 1)
            }
            node.geometry!.firstMaterial!.diffuse.contents = color
        }
        let flashColors = [flashRed, flashGreen, flashBlue]
        
        var changeColor: SCNAction!
        if (defaultColor == 3) {
            changeColor = SCNAction.repeatForever(flashColors[Int(arc4random_uniform(3))])
        } else {
            changeColor = SCNAction.repeatForever(flashColors[defaultColor])
        }
        
        keypointNode.runAction(changeColor)
        sceneView.scene.rootNode.addChildNode(keypointNode)
    }
    
    func getCameraCoordinates(sceneView: ARSCNView) -> LocationInfo {
        let cameraTransform = sceneView.session.currentFrame?.camera.transform
        let coordinates = MDLTransform(matrix: cameraTransform!)
        
        return LocationInfo(x: coordinates.translation.x,
                            y: coordinates.translation.y,
                            z: coordinates.translation.z,
                            yaw: coordinates.rotation.y)
    }
    
    func getRealCoordinates(sceneView: ARSCNView) -> CurrentCoordinateInfo {
        let x = SCNMatrix4((sceneView.session.currentFrame?.camera.transform)!).m41
        let y = SCNMatrix4((sceneView.session.currentFrame?.camera.transform)!).m42
        let z = SCNMatrix4((sceneView.session.currentFrame?.camera.transform)!).m43
        
        let yaw = sceneView.session.currentFrame?.camera.eulerAngles.y
        let scn = SCNMatrix4((sceneView.session.currentFrame?.camera.transform)!)
        let transMatrix = Matrix3([scn.m11, scn.m12, scn.m13,
                                   scn.m21, scn.m22, scn.m23,
                                   scn.m31, scn.m32, scn.m33])
        
        if (navigationMode) {
            navigationData.append([[scn.m11, scn.m12, scn.m13, scn.m14],
                             [scn.m21, scn.m22, scn.m23, scn.m24],
                             [scn.m31, scn.m32, scn.m33, scn.m34],
                             [scn.m41, scn.m42, scn.m43, scn.m44]])
            navigationDataTime.append(-dataTimer.timeIntervalSinceNow)
        } else {
            pathData.append([[scn.m11, scn.m12, scn.m13, scn.m14],
                             [scn.m21, scn.m22, scn.m23, scn.m24],
                             [scn.m31, scn.m32, scn.m33, scn.m34],
                             [scn.m41, scn.m42, scn.m43, scn.m44]])
            pathDataTime.append(-dataTimer.timeIntervalSinceNow)
        }
        
        return CurrentCoordinateInfo(LocationInfo(x: x, y: y, z: z, yaw: yaw!), transMatrix: transMatrix)
    }
}

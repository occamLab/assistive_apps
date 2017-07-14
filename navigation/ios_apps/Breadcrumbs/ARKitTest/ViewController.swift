//
//  ViewController.swift
//  ARKitTest
//
//  Created by Chris Seonghwan Yoon on 7/10/17.
//  Copyright © 2017 Stanford. All rights reserved.
//

import UIKit
import ARKit
import SceneKit
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
        createARSession()
        drawUI()
        addGestures()
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
        let tapGestureRecognizer = UITapGestureRecognizer(target: self, action: #selector(announceDirections))
        tapGestureRecognizer.numberOfTapsRequired = 2
        self.view.addGestureRecognizer(tapGestureRecognizer)
    }
    
    func drawUI() {
        // button that gives direction to the nearist keypoint
        getDirectionButton = UIButton(frame: CGRect(x: 0, y: 0, width: buttonFrameWidth, height: yOriginOfButtonFrame))
        getDirectionButton.isAccessibilityElement = true
        getDirectionButton.accessibilityLabel = "Get Directions"
        getDirectionButton.addTarget(self, action: #selector(announceDirections), for: .touchUpInside)
        
        // textlabel that displys directions
        directionText = UILabel(frame: CGRect(x: 0, y: (yOriginOfButtonFrame + textLabelBuffer), width: buttonFrameWidth, height: buttonFrameHeight*(1/6)))
        directionText.textColor = UIColor.white
        directionText.textAlignment = .center
        directionText.isAccessibilityElement = false
        
        // Record Path button container
        recordPathView = UIView(frame: CGRect(x: 0, y: yOriginOfButtonFrame, width: buttonFrameWidth, height: buttonFrameHeight))
        recordPathView.backgroundColor = UIColor.black.withAlphaComponent(0.4)
        addButtons(buttonView: recordPathView, buttonViewType: .recordPath)
        
        // Stop Recording button container
        stopRecordingView = UIView(frame: CGRect(x: 0, y: yOriginOfButtonFrame, width: buttonFrameWidth, height: buttonFrameHeight))
        stopRecordingView.backgroundColor = UIColor.black.withAlphaComponent(0.4)
        addButtons(buttonView: stopRecordingView, buttonViewType: .stopRecording)
        
        // Start Navigation button container
        startNavigationView = UIView(frame: CGRect(x: 0, y: yOriginOfButtonFrame, width: buttonFrameWidth, height: buttonFrameHeight))
        startNavigationView.backgroundColor = UIColor.black.withAlphaComponent(0.4)
        addButtons(buttonView: startNavigationView, buttonViewType: .startNavigation)
        
        // Stop Navigation button container
        stopNavigationView = UIView(frame: CGRect(x: 0, y: yOriginOfButtonFrame, width: buttonFrameWidth, height: buttonFrameHeight))
        stopNavigationView.backgroundColor = UIColor.black.withAlphaComponent(0.4)
        addButtons(buttonView: stopNavigationView, buttonViewType: .stopNavigation)
        
        self.view.addSubview(recordPathView)
        self.view.addSubview(stopRecordingView)
        self.view.addSubview(startNavigationView)
        self.view.addSubview(stopNavigationView)
        self.view.addSubview(directionText)
        self.view.addSubview(getDirectionButton)
        showRecordPathButton()
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
        // turn off annoucementTimer
        if (announcementTimerOn) {
            announcementTimer.invalidate()
        }
        
        recordPathView.isHidden = false
        stopRecordingView.isHidden = true
        startNavigationView.isHidden = true
        stopNavigationView.isHidden = true
        getDirectionButton.isHidden = true
        navigationMode = false
        currentButton = .recordPath
        updateDirectionText("Press to record path", size: 16, distance: false)
    }
    
    /*
     * display STOP RECORDIN button/hide all other views
     */
    func showStopRecordingButton() {
        recordPathView.isHidden = true
        stopRecordingView.isHidden = false
        startNavigationView.isHidden = true
        stopNavigationView.isHidden = true
        getDirectionButton.isHidden = true
        navigationMode = false
        currentButton = .stopRecording
        updateDirectionText("Recording Path", size: 16, distance: false)
    }
    
    /*
     * display START NAVIGATION button/hide all other views
     */
    func showStartNavigationButton() {
        recordPathView.isHidden = true
        stopRecordingView.isHidden = true
        startNavigationView.isHidden = false
        stopNavigationView.isHidden = true
        getDirectionButton.isHidden = true
        navigationMode = false
        currentButton = .startNavigation
        updateDirectionText("Press to start navigation", size: 16, distance: false)
    }
    
    /*
     * display STOP NAVIGATION button/hide all other views
     */
    func showStopNavigationButton() {
        recordPathView.isHidden = true
        stopRecordingView.isHidden = true
        startNavigationView.isHidden = true
        stopNavigationView.isHidden = false
        getDirectionButton.isHidden = false
        navigationMode = true
        currentButton = .stopNavigation
    }
    
    /*
     * update directionText UILabel given text string and font size
     * distance Bool used to determine whether to add string "meters" to direction text
     */
    func updateDirectionText(_ text: String, size: CGFloat, distance: Bool) {
        directionText.fadeTransition(0.4)
        directionText.font = directionText.font.withSize(size)
        var altText = ""
        if(distance) {
            directionText.text = text + "m"
            altText = text + " meters"
        } else {
            directionText.text = text
            altText = text
        }
        UIAccessibilityPostNotification(UIAccessibilityAnnouncementNotification, altText)
    }
    
    /*                  BreadCrumbs main logic                 */
    /*---------------------------------------------------------*/
    
    var crumbs = [LocationInfo]()       // List of crumbs dropped when recording path
    var keypoints = [KeypointInfo]()    // List of keypoints calculated after path completion
    var keypointNode: SCNNode!          // SCNNode of the next keypoint
    
    /* Timers for background functions */
    var droppingCrumbs: Timer!
    var followingCrumbs: Timer!
    var announcementTimer: Timer!
    var announcementTimerOn = false
    
    var nav = Navigation()                  // Navigation calculation class
    var navigationMode: Bool = false        // navigation flag 
    
    @IBOutlet weak var sceneView: ARSCNView!
    
    @objc func recordPath() {
        showStopRecordingButton()
        droppingCrumbs = Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: #selector(dropCrum), userInfo: nil, repeats: true)
    }
    
    @objc func stopRecording(_ sender: UIButton) {
        droppingCrumbs.invalidate()
        showStartNavigationButton()
    }
    
    @objc func startNavigation(_ sender: UIButton) {
        showStopNavigationButton()
        let path = PathFinder(crums: crumbs.reversed())
        keypoints = path.keypoints
        
        renderCube(keypoints[0].location)
        
        followingCrumbs = Timer.scheduledTimer(timeInterval: 0.2, target: self, selector: (#selector(followCrum)), userInfo: nil, repeats: true)
    }
    
    @objc func stopNavigation(_ sender: UIButton) {
        followingCrumbs.invalidate()
        crumbs = []
        showRecordPathButton()
    }
    
    @objc func dropCrum() {
        let curLocation = getRealCoordinates(sceneView: sceneView)
        crumbs.append(curLocation)
    }
    
    @objc func announceDirections() {
        if (navigationMode) {
            let curLocation = getRealCoordinates(sceneView: sceneView)
            let directionToNextKeypoint = getDirectionToNextKeypoint(currentLocation: curLocation)
            
            let dir = Directions[directionToNextKeypoint.clockDirection]! + "for \(directionToNextKeypoint.distance)"
            updateDirectionText(dir, size: 16, distance: true)
        }
    }
    
    @objc func followCrum() {
        let curLocation = getRealCoordinates(sceneView: sceneView)
        let directionToNextKeypoint = getDirectionToNextKeypoint(currentLocation: curLocation)
        
        if(directionToNextKeypoint.targetState == PositionState.atTarget) {
            if (keypoints.count > 1) {
                keypointNode.removeFromParentNode()
                keypoints.remove(at: 0)
                renderCube(keypoints[0].location)
                speakDirection(currentLocation: curLocation)
            } else {
                keypointNode.removeFromParentNode()
                announceArrival()
                followingCrumbs.invalidate()
            }
        }
        
    }
    
    func getDirectionToNextKeypoint(currentLocation: LocationInfo) -> DirectionInfo {
        return nav.getDirections(currentLocation: currentLocation, nextKeypoint: keypoints[0])
    }
    
    func speakDirection(currentLocation: LocationInfo) {
        let xzNorm = sqrtf(powf(currentLocation.x - keypoints[0].location.x, 2) + powf(currentLocation.z - keypoints[0].location.z, 2))
        let slope = (keypoints[0].location.y - currentLocation.y) / xzNorm
        let direction = getDirectionToNextKeypoint(currentLocation: currentLocation)
        var dir = ""
        
        if(slope > 0.3) { // Go upstairs
            dir = "\(Directions[direction.clockDirection]!) and proceed upstairs"
            updateDirectionText(dir, size: 14, distance: false)
        } else if (slope < -0.3) { // Go downstairs
            dir = "\(Directions[direction.clockDirection]!) and proceed downstairs"
            updateDirectionText(dir, size: 14, distance: false)
        } else { // nromal directions
            dir = Directions[direction.clockDirection]!
            updateDirectionText(dir, size: 16, distance:  false)
        }
    }
    
    func announceArrival() {
        crumbs = []
        updateDirectionText("You have arrived!", size: 16, distance: false)
        announcementTimer = Timer.scheduledTimer(timeInterval: 1, target: self, selector: (#selector(showRecordPathButton)), userInfo: nil, repeats: false)
    }
    
    func renderCube(_ location: LocationInfo) {
        keypointNode = SCNNode(geometry: SCNBox(width: 0.3, height: 0.3, length: 0.05, chamferRadius: 0.05))
        keypointNode.opacity = 0.5
        
        keypointNode.position = SCNVector3(location.x, location.y, location.z)
        keypointNode.rotation = SCNVector4(0, 1, 0, location.a)
        
        sceneView.scene.rootNode.addChildNode(keypointNode)
    }
    
    func getCameraCoordinates(sceneView: ARSCNView) -> LocationInfo {
        let cameraTransform = sceneView.session.currentFrame?.camera.transform
        let coordinates = MDLTransform(matrix: cameraTransform!)
        
        return LocationInfo(x: coordinates.translation.x,
                            y: coordinates.translation.y,
                            z: coordinates.translation.z,
                            a: coordinates.rotation.y)
    }
    
    func getRealCoordinates(sceneView: ARSCNView) -> LocationInfo{
        return LocationInfo(x: SCNMatrix4((sceneView.session.currentFrame?.camera.transform)!).m41,
                            y: SCNMatrix4((sceneView.session.currentFrame?.camera.transform)!).m42,
                            z: SCNMatrix4((sceneView.session.currentFrame?.camera.transform)!).m43,
                            a: (sceneView.session.currentFrame?.camera.eulerAngles.y)!)
    }
    
}

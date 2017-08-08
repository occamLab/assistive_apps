//
//  ViewController.swift
//  gpsvionavigation
//
//  Created by SCOPE on 8/8/17.
//  Copyright © 2017 occamlab. All rights reserved.
//

import UIKit
import SceneKit
import ARKit

class ViewController: UIViewController, ARSCNViewDelegate {

    @IBOutlet var sceneView: ARSCNView!
    
    /* UI setup */
    
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
    
    var recordPathView: UIView!
    var stopRecordingView: UIView!
    
    // State of button views
    enum ButtonViewType {
        case recordPath
        case stopRecording
    }
    
    // current button in display
    var currentButton = ButtonViewType.recordPath
    
    
    var crumbs: [LocationInfo]!         // List of crumbs dropped when recording path
    var pathData: [Array<Any>]!
    var pathDataTime: [Double]!
    
    var dataTimer: Date!
    var droppingCrumbs: Timer!
    var announcementTimer: Timer!
    
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
        announcementTimer = Timer.scheduledTimer(timeInterval: 1, target: self, selector: (#selector(showRecordPathButton)), userInfo: nil, repeats: false)
    }
    
    @objc func dropCrum() {
        let curLocation = getRealCoordinates(sceneView: sceneView).location
        crumbs.append(curLocation)
    }
    
    /*
     * display RECORD PATH button/hide all other views
     */
    @objc func showRecordPathButton() {
        recordPathView.isHidden = false
        stopRecordingView.isHidden = true;
        currentButton = .recordPath
    }
    
    /*
     * display STOP RECORDIN button/hide all other views
     */
    @objc func showStopRecordingButton() {
        recordPathView.isHidden = true
        stopRecordingView.isHidden = false
        currentButton = .stopRecording
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Set the view's delegate
        sceneView.delegate = self
        
        // Show statistics such as fps and timing information
        sceneView.showsStatistics = true
        
        // Create a new scene
        let scene = SCNScene(named: "art.scnassets/ship.scn")!
        
        // Set the scene to the view
        sceneView.scene = scene
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a session configuration
        let configuration = ARWorldTrackingSessionConfiguration()
        
        // Run the view's session
        sceneView.session.run(configuration)
        
        drawUI()
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        // Pause the view's session
        sceneView.session.pause()
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Release any cached data, images, etc that aren't in use.
    }

    func drawUI() {
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
        
        self.view.addSubview(recordPathView)
        self.view.addSubview(stopRecordingView)
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
            button.accessibilityLabel = "Re-cord Path"
            button.addTarget(self, action: #selector(recordPath), for: .touchUpInside)
        case.stopRecording:
            let buttonImage = UIImage(named: "StopRecording")
            button.setImage(buttonImage, for: .normal)
            button.accessibilityLabel = "Stop recording"
            button.addTarget(self, action: #selector(stopRecording), for: .touchUpInside)
        }
        
        buttonView.addSubview(button)
    }
    
    // MARK: - ARSCNViewDelegate
    
/*
    // Override to create and configure nodes for anchors added to the view's session.
    func renderer(_ renderer: SCNSceneRenderer, nodeFor anchor: ARAnchor) -> SCNNode? {
        let node = SCNNode()
     
        return node
    }
*/
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        // Present an error message to the user
        
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        // Inform the user that the session has been interrupted, for example, by presenting an overlay
        
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        // Reset tracking and/or remove existing anchors if consistent tracking is required
        
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
        

        pathData.append([[scn.m11, scn.m12, scn.m13, scn.m14],
                         [scn.m21, scn.m22, scn.m23, scn.m24],
                         [scn.m31, scn.m32, scn.m33, scn.m34],
                         [scn.m41, scn.m42, scn.m43, scn.m44]])
        pathDataTime.append(-dataTimer.timeIntervalSinceNow)
        
        return CurrentCoordinateInfo(LocationInfo(x: x, y: y, z: z, yaw: yaw!), transMatrix: transMatrix)
    }
}

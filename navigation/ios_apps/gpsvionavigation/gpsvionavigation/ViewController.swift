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
import SwiftyJSON
import CoreLocation
import GoogleMaps
import GooglePlaces

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
    var gpsButtonView: UIView!
    
    // State of button views
    enum ButtonViewType {
        case recordPath
        case stopRecording
//        case dropGPSPin
    }
    
    // current button in display
    var currentButton = ButtonViewType.recordPath
    
    var viocrumbs: [LocationInfo]!         // List of crumbs dropped when recording path
    var gpscrumbs: [CLLocationCoordinate2D]!
    var pathData: [Array<Any>]!
    var pathDataTime: [Double]!
    
    var dataTimer: Date!
    var droppingCrumbs: Timer!
    var announcementTimer: Timer!
    
    //* GPS attributes //
    var locationManager = CLLocationManager()
    var currentLocation: CLLocation?

    
    @objc func recordPath() {
        viocrumbs = []
        gpscrumbs = []
        pathData = []
        pathDataTime = []
        dataTimer = Date()
        announcementTimer = Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: (#selector(showStopRecordingButton)), userInfo: nil, repeats: false)
        droppingCrumbs = Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: #selector(dropCrum), userInfo: nil, repeats: true)
    }
    
    @objc func stopRecording(_ sender: UIButton) {
        droppingCrumbs.invalidate()
        announcementTimer = Timer.scheduledTimer(timeInterval: 1, target: self, selector: (#selector(showRecordPathButton)), userInfo: nil, repeats: false)
        
        let json: JSON = [
            "vio_crumbs": serializeVIOCrumbs(),
            "gps_crumbs": serializeGPSCrumbs()
        ]
        if let string = json.rawString() {
            print(string)
            do {
                // Write contents to file
                // We can... but I have no clue how to access this later.
                try string.write(toFile: NSTemporaryDirectory() + "crumbdata.json", atomically: false, encoding: String.Encoding.utf8)
            }
            catch let error as NSError {
                print("Ooops! Something went wrong: \(error)")
            }
        }
    }
    
    /*
     * Turns the [LocationInfo] datastructure into a serializable format for JSON
     */
    func serializeVIOCrumbs() -> [[Float]] {
        var vio_list: [[Float]] = []
        if let crumbs = viocrumbs
        {
            for object in crumbs {
                vio_list.append([object.x, object.y, object.z, object.yaw])
            }
        }
        return vio_list
    }
    
    /*
     * Turns the [CLLocationCoordinate2D] datastructure into a serializable format for JSON
     */
    func serializeGPSCrumbs() -> [[Double]] {
        var gps_list: [[Double]] = []
        if let crumbs = gpscrumbs
        {
            for object in crumbs {
                gps_list.append([object.latitude, object.longitude])
            }
        }
        return gps_list
    }
    
    @objc func dropCrum() {
        let curLocation = getRealCoordinates(sceneView: sceneView).location
        viocrumbs.append(curLocation)
//        print(curLocation)
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
        sceneView.showsStatistics = false
        
        // Create a new scene
        let scene = SCNScene(named: "art.scnassets/ship.scn")!
        
        // Set the scene to the view
        sceneView.scene = scene
        
        // Initialize the location manager.
        locationManager = CLLocationManager()
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
//        locationManager.requestAlwaysAuthorization()
        locationManager.requestWhenInUseAuthorization()
        locationManager.distanceFilter = kCLDistanceFilterNone
        locationManager.startUpdatingLocation()
        locationManager.delegate = self
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
        
//        gpsButtonView = UIView(frame: CGRect(x: 0, y: yOriginOfButtonFrame, width: buttonFrameWidth, height: buttonFrameHeight))
//        gpsButtonView.backgroundColor = UIColor.black.withAlphaComponent(0.4)
//        gpsButtonView.isHidden = false
//        addButtons(buttonView: gpsButtonView, buttonViewType: .dropGPSPin)
//
        self.view.addSubview(recordPathView)
        self.view.addSubview(stopRecordingView)
//        self.view.addSubview(gpsButtonView)
        
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
        

        
        // Adds custom button labels
        switch buttonViewType {
        case .recordPath:
            let buttonImage = UIImage(named: "StartRecording")
            button.center.x = buttonView.bounds.size.width * (1/4)
            button.center.y = buttonView.bounds.size.height * (6/10)
            button.setImage(buttonImage, for: .normal)
            button.accessibilityLabel = "Re-cord Path"
            button.addTarget(self, action: #selector(recordPath), for: .touchUpInside)
        case.stopRecording:
            let buttonImage = UIImage(named: "StopRecording")
            button.center.x = buttonView.bounds.size.width * (1/4)
            button.center.y = buttonView.bounds.size.height * (6/10)
            button.setImage(buttonImage, for: .normal)
            button.accessibilityLabel = "Stop recording"
            button.addTarget(self, action: #selector(stopRecording), for: .touchUpInside)
        }
//        case.dropGPSPin:
//            let buttonImage = UIImage(named: "GPSButton")
//            button.center.x = buttonView.bounds.size.width * (3/4)
//            button.center.y = buttonView.bounds.size.height * (6/10)
//            button.setImage(buttonImage, for: .normal)
//            button.accessibilityLabel = "Drop GPS Pin"
////            button.addTarget(self, action: #selector(dropGPS), for: .touchUpInside)
//        }
        
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

// Delegates to handle events for the location manager.
extension ViewController: CLLocationManagerDelegate {

    
    // Handle incoming location events.
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        let location: CLLocation = locations.last!
        print("Location: \(location)")
        
        if currentButton == .stopRecording {
            gpscrumbs.append(location.coordinate)
        }
    }
    
    // Handle authorization for the location manager.
    func locationManager(_ manager: CLLocationManager, didChangeAuthorization status: CLAuthorizationStatus) {
        switch status {
        case .restricted:
            print("Location access was restricted.")
        case .denied:
            print("User denied access to location.")
        case .notDetermined:
            print("Location status not determined.")
        case .authorizedAlways: fallthrough
        case .authorizedWhenInUse:
            print("Location status is OK.")
        }
    }
    
    // Handle location manager errors.
    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        manager.stopUpdatingLocation()
        print("Error: \(error)")
    }
}


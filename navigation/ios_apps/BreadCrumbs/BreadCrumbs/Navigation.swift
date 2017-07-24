//
//  Navigation.swift
//  ARKitTest
//
//  Created by Chris Seonghwan Yoon & Jeremy Ryan on 7/11/17.
//
//  Navigation class that provides direction information given 2 LocationInfo position
//

import Foundation

public enum PositionState {
    case notAtTarget
    case atTarget
    case pastTarget
}

public struct DirectionInfo {
    public var distance: Float
    public var clockDirection: Int
    
    public var targetState = PositionState.notAtTarget
    
    public init(distance: Float, clockDirection: Int) {
        self.distance = distance
        self.clockDirection = clockDirection
    }
}

public let Directions = [12: "Continue straight",
                         1: "Slight right towards 1 o'clock",
                         2: "Slight right towards 2 o'clock",
                         3: "Turn right",
                         4: "Turn towards 4 o'clock",
                         5: "Turn around towards 5 o'clock",
                         6: "Turn around towards 6 o'clock",
                         7: "Turn around towards 7 o'clock",
                         8: "Turn towards 8 o'clock",
                         9: "Turn left",
                         10: "Slight left towards 10 o'clock",
                         11: "Slight left towards 11 o'clock"]

/* Keypoint target dimensions */
public var targetWidth: Scalar = 2
public var targetDepth: Scalar = 0.5
public var targetHeight: Scalar = 3

class Navigation {
    public func getDirections(currentLocation: CurrentCoordinateInfo, nextKeypoint: KeypointInfo) -> DirectionInfo {
        let zVector = Vector3.z * currentLocation.transformMatrix
        let yVector = Vector3.x * currentLocation.transformMatrix
        var trueVector: Vector3!
        
        if (abs(zVector.y) < abs(yVector.y)) {
            trueVector = zVector * Matrix3([1, 0, 0, 0, 0, 0, 0, 0, 1])
            print("Phone is upright.")
        } else {
            trueVector = yVector * Matrix3([1, 0, 0, 0, 0, 0, 0, 0, 1])
            print("Phone is flat.")
        }
        let trueYaw = atan2f(trueVector.x, trueVector.z)
        
        let dist = sqrtf(powf((currentLocation.location.x - nextKeypoint.location.x), 2) +
            powf((currentLocation.location.z - nextKeypoint.location.z), 2))
        
        let angle = atan2f((currentLocation.location.x - nextKeypoint.location.x), (currentLocation.location.z-nextKeypoint.location.z))
        
        let angleDiff = getAngleDiff(angle1: trueYaw, angle2: angle)
        
        let clockDir = getClockDirections(angle: angleDiff)
        
        let xDiff = Vector3([currentLocation.location.x - nextKeypoint.location.x,
                             currentLocation.location.y - nextKeypoint.location.y,
                             currentLocation.location.z - nextKeypoint.location.z]).dot(nextKeypoint.orientation)
        let yDiff = Vector3([currentLocation.location.x - nextKeypoint.location.x,
                             currentLocation.location.y - nextKeypoint.location.y,
                             currentLocation.location.z - nextKeypoint.location.z]).dot(Vector3.y)
        let zDiff = Vector3([currentLocation.location.x - nextKeypoint.location.x,
                             currentLocation.location.y - nextKeypoint.location.y,
                             currentLocation.location.z - nextKeypoint.location.z]).dot(nextKeypoint.orientation.cross(Vector3.y))
        
        var direction = DirectionInfo(distance: dist, clockDirection: clockDir)
        
        if (xDiff <= targetDepth && yDiff <= targetHeight && zDiff <= targetWidth) {
            direction.targetState = .atTarget
        } else {
            direction.targetState = .notAtTarget
        }
        
        return direction
    }
    
    private func getClockDirections(angle: Float) -> Int {
        let a = (angle * (6/Float.pi)) + 12.5
        
        let clockDir = Int(a) % 12
        return clockDir == 0 ? 12 : clockDir
    }
    
    private func getAngleDiff(angle1: Float, angle2: Float) -> Float {
        let a = angleNormalize(angle: angle1)
        let b = angleNormalize(angle: angle2)
        
        let d1 = a-b
        var d2 = 2*Float.pi - abs(d1)
        if (d1 > 0) {
            d2 = d2 * (-1)
        }
        return abs(d1) < abs(d2) ? d1 : d2
    }
    
    private func angleNormalize(angle: Float) -> Float {
        return atan2f(sinf(angle), cosf(angle))
    }
    
    private func roundToTenths(n: Float) -> Float {
        return roundf(10 * n)/10
    }
}



//
//  DataPersistance.swift
//  Clew
//
//  Created by Khang Vu on 3/14/19.
//  Copyright © 2019 OccamLab. All rights reserved.
//

import Foundation
import ARKit

@available(iOS 12.0, *)
class DataPersistance {
    
    var routes = [SavedRoute]()

    init() {
        guard let newRoutes = NSKeyedUnarchiver.unarchiveObject(withFile: self.getRoutesURL().path) as? [SavedRoute] else {return}
        self.routes = newRoutes
    }
    
    func archive(route: SavedRoute, worldMap: ARWorldMap) throws {
        // Save route to the route list
        self.routes.append(route)
        NSKeyedArchiver.archiveRootObject(self.routes, toFile: self.getRoutesURL().path)
        // Save the world map corresponding to the route
        let data = try NSKeyedArchiver.archivedData(withRootObject: worldMap, requiringSecureCoding: true)
        try data.write(to: self.getWorldMapURL(id: route.id as String), options: [.atomic])
    }
    
    func unarchive(routeId: String) -> ARWorldMap? {
        var data: Data
        do {
            data = try Data(contentsOf: getWorldMapURL(id: routeId))
        } catch {
            print("Error retrieving world map data.")
            return nil
        }
        guard let unarchievedObject = try? NSKeyedUnarchiver.unarchivedObject(ofClass: ARWorldMap.self, from: data),
            let worldMap = unarchievedObject else { return nil }
        return worldMap
    }
    
    func delete(route: SavedRoute) throws {
        // Remove route from the route list
        self.routes = self.routes.filter { $0.id != route.id }
        NSKeyedArchiver.archiveRootObject(self.routes, toFile: self.getRoutesURL().path)
        // Remove the world map corresponding to the route
        try FileManager().removeItem(atPath: self.getWorldMapURL(id: route.id as String).path)
    }
    
    private func getURL(url: String) -> URL {
        return FileManager().urls(for: .documentDirectory, in: .userDomainMask).first!.appendingPathComponent(url)
    }
    
    private func getWorldMapURL(id: String) -> URL {
        return getURL(url: id)
    }
    
    private func getRoutesURL() -> URL {
        return getURL(url: "routeList")
    }
    
}

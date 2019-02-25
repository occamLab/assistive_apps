//
//  RoutesViewController.swift
//  Clew
//
//  Created by Khang Vu on 2/22/19.
//  Copyright © 2019 OccamLab. All rights reserved.
//

import Foundation
import WebKit

class RoutesViewController : UIViewController, UITableViewDataSource, UITableViewDelegate {
    
    @IBOutlet weak var tableView: UITableView!
    
    var routes = [SavedRoute]()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        tableView.dataSource = self
    }
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
//        tableView.deselectRow(at: indexPath, animated: true)
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = tableView.dequeueReusableCell(withIdentifier: "clew.RouteTableViewCell", for: indexPath) as! RouteTableViewCell
        cell.nameLabel.text = routes[indexPath.row].name
        let df = DateFormatter()
        df.dateFormat = "MM/DD/YYYY"
        cell.dateCreatedLabel.text = df.string(from: routes[indexPath.row].dateCreated)
        return cell
    }
    
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        return routes.count
    }
    
    func updateRoutes(routes: [String : SavedRoute]) {
        var tempRoutes = [SavedRoute]()
        for (_, savedRoute) in routes {
            tempRoutes.append(savedRoute)
        }
        self.routes = tempRoutes.sorted(by: { $0.dateCreated > $1.dateCreated })
    }
}

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
    var rootViewController: ViewController?
    var routes = [SavedRoute]()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.tableView.delegate = self
        self.tableView.dataSource = self
    }
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        self.tableView.deselectRow(at: indexPath, animated: true)
        self.rootViewController?.onRouteTableViewCellClicked(routeId: self.routes[indexPath.row].id)
        self.dismiss(animated: true, completion: nil)
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let df = DateFormatter()
        df.dateFormat = "MM/DD/YYYY"
        let cell = tableView.dequeueReusableCell(withIdentifier: "clew.RouteTableViewCell", for: indexPath) as! RouteTableViewCell
        cell.nameLabel.text = routes[indexPath.row].name
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

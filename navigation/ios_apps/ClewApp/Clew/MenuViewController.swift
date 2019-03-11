//
//  MenuViewController.swift
//  Clew
//
//  Created by Khang Vu on 2/27/19.
//  Copyright © 2019 OccamLab. All rights reserved.
//

import UIKit

@available(iOS 12.0, *)
class MenuViewController: UIViewController, UITableViewDataSource, UITableViewDelegate {
    
    @IBOutlet weak var menuTable: UITableView!
    var rootViewController: ViewController?
    var routes = [SavedRoute]()
    let menuItems = ["Load saved routes", "Resume recording"]
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.menuTable.delegate = self
        self.menuTable.dataSource = self
    }
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        self.menuTable.deselectRow(at: indexPath, animated: true)
    }
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "showRoutesViewControllerSegue" {
            let cell = sender as! UITableViewCell
            if let indexPath = self.menuTable.indexPath(for: cell) {
                if indexPath.row <= 1 {
                    let routesVC = segue.destination as! RoutesViewController
                    routesVC.vcType = indexPath.row == 0 ? ViewControllerType.loadSavedRoutes : ViewControllerType.resumeRecording
                    routesVC.title = menuItems[indexPath.row]
                    routesVC.routes = self.routes
                    routesVC.rootViewController = self.rootViewController
                }
            }
        }
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = tableView.dequeueReusableCell(withIdentifier: "clew.MenuTableViewCell", for: indexPath)
        cell.textLabel?.text = menuItems[indexPath.row]
        return cell
    }
    
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        return menuItems.count
    }
    
    func loadRoutes(routes: [String : SavedRoute]) {
        var tempRoutes = [SavedRoute]()
        for (_, savedRoute) in routes {
            tempRoutes.append(savedRoute)
        }
        self.routes = tempRoutes.sorted(by: { $0.dateCreated as Date > $1.dateCreated as Date})
    }
}

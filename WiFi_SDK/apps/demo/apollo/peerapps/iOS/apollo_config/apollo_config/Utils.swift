//
//  Utils.swift
//  apollo_config

/*
 * $ Copyright Broadcom Corporation $
 */

import Foundation
import UIKit

class Utils {
    
    class func sendNotification(note:String, soundName:String) {
        let notification = UILocalNotification()
        notification.fireDate = NSDate(timeIntervalSinceNow: 1)
        notification.hasAction = false
        notification.alertBody = note
        notification.timeZone = NSTimeZone.defaultTimeZone()
        notification.soundName = soundName
        UIApplication.sharedApplication().scheduleLocalNotification(notification)
    }
    
    class func showAlert(message:String) {
        let alert = UIAlertView()
        alert.title = "Notification"
        alert.message = message
        alert.addButtonWithTitle("OK")
        alert.show()
    }
    
}
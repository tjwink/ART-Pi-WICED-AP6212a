//
//  Logger.swift
//  testing
//
//  Copyright © 2015 broadcom. All rights reserved.
//

import Foundation

open class Logger {
    
    open class func debug(_ message:AnyObject) {
        //#if DEBUG
            print("\(message)")
        //#endif
    }
    
}
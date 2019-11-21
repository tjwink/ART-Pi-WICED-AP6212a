//
//  Logger.swift
//  apollo_config

/*
 * $ Copyright Broadcom Corporation $
 */
 
import Foundation

public class Logger {
    
    public class func debug(message:AnyObject) {
        #if DEBUG
            print("\(message)")
        #endif
    }
    
}
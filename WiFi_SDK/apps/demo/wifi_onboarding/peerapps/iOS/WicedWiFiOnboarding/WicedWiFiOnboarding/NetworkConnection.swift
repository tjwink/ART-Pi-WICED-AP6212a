//
//  NetworkConnection.swift
//  WicedWiFiIntroducer
//
//  Created by pradeep govindaswamy on 7/18/17.
//  Copyright © 2017 bluth. All rights reserved.
//

import Foundation
import Alamofire
class NetworkConnection {
  //  let developmentDomain = Config.developmentDomain // "api.myappdev.com"
    let productionDomain = "wiced.com" // "api.myappprod.com"
    let certificateFilename = "amqps_root_cacert" // "godaddy"
    let certificateExtension = "der"// "der"
    let useSSL = true
    var manager: SessionManager!
    var serverTrustPolicies: [String : ServerTrustPolicy] = [String:ServerTrustPolicy]()
    static let sharedManager = NetworkConnection()
    
    
    init(){
        if useSSL {
            manager = initSafeManager()
        } else {
            manager = initUnsafeManager()
        }
    }
    
    //USED FOR SITES WITH CERTIFICATE, OTHERWISE .DisableEvaluation
    func initSafeManager() -> SessionManager {
        setServerTrustPolicies()
        
        manager = SessionManager(configuration: URLSessionConfiguration.default, delegate: SessionDelegate(), serverTrustPolicyManager: ServerTrustPolicyManager(policies: serverTrustPolicies))
        
        return manager
    }
    
    //USED FOR SITES WITHOUT CERTIFICATE, DOESN'T CHECK FOR CERTIFICATE
    func initUnsafeManager() -> SessionManager {
        manager = Alamofire.SessionManager.default
        
        manager.delegate.sessionDidReceiveChallenge = { session, challenge in
            var disposition: URLSession.AuthChallengeDisposition = .performDefaultHandling
            var credential: URLCredential?
            
            if challenge.protectionSpace.authenticationMethod == NSURLAuthenticationMethodServerTrust {
                disposition = URLSession.AuthChallengeDisposition.useCredential
                credential = URLCredential(trust: challenge.protectionSpace.serverTrust!)     //URLCredential(forTrust: challenge.protectionSpace.serverTrust!)
            } else {
                if challenge.previousFailureCount > 0 {
                    disposition = .cancelAuthenticationChallenge
                } else {
                    credential = self.manager.session.configuration.urlCredentialStorage?.defaultCredential(for: challenge.protectionSpace)
                    
                    if credential != nil {
                        disposition = .useCredential
                    }
                }
            }
            
            return (disposition, credential)
        }
        
        return manager
    }
    
    func setServerTrustPolicies() {
        let pathToCert = Bundle.main.path(forResource: certificateFilename, ofType: certificateExtension)
        let localCertificate:Data = try! Data(contentsOf: URL(fileURLWithPath: pathToCert!))
        
        let serverTrustPolicies: [String: ServerTrustPolicy] = [
            productionDomain: .pinCertificates(
                certificates: [SecCertificateCreateWithData(nil, localCertificate as CFData)!],
                validateCertificateChain: true,
                validateHost: true
            )
       //     developmentDomain: .disableEvaluation
        ]
        
        self.serverTrustPolicies = serverTrustPolicies
    }
    
    static func addAuthorizationHeader (_ token: String, tokenType: String) -> [String : String] {
        let headers = [
            "Authorization": tokenType + " " + token
        ]
        
        return headers
    }
    
}

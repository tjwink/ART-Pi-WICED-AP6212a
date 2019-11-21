

import Foundation
import Alamofire
class RestHandler : NSObject {
    
    func makGetRequest() {
        Alamofire.request("https://www.wiced.com/get")
    }
    
    func makePostRequest() {
        let parameters: Parameters = [
            "HomeAP": DataManager.sharedInstance().homeAP,
            "Password":DataManager.sharedInstance().homeAPpwd
        ]
        
        // Both calls are equivalent
        let req = Alamofire.request("https://www.wiced.com/post", method: .post, parameters: parameters, encoding: JSONEncoding.default)
        
        print("Status : ", (req.response?.statusCode) ?? "Default status");
        
        
    }

}



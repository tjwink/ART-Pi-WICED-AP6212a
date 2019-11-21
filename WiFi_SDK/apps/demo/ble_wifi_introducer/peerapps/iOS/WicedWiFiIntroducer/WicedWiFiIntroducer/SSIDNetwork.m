#import "SSIDNetwork.h"
#import <SystemConfiguration/CaptiveNetwork.h>

@interface SSIDNetwork ()

@end

@implementation SSIDNetwork


- (NSString*)getNetworkSSID {
    
    NSString *wifiName = nil;
    NSArray *interFaceNames = (__bridge_transfer id)CNCopySupportedInterfaces();
    
    for (NSString *name in interFaceNames) {
        NSDictionary *info = (__bridge_transfer id)CNCopyCurrentNetworkInfo((__bridge CFStringRef)name);


        if (info[@"SSID"]) {
            wifiName = info[@"SSID"];
        }
    }
    if ( wifiName != nil) {
        return wifiName;
    }
    else
        return nil;
    
}
@end
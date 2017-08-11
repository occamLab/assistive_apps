# GPS + Visual Inertial Odometry (VIO) Navigation

## Installation

You need Cocoapods!

Once you have it, you'll run `pod install`

We ran into some issues with iOS 11, since AWSCore iOS SDK is not up to date for iOS 11.

There might be some errors in AWSCore: you'll have to manually change some lines related to this error.

Error:
https://github.com/CocoaLumberjack/CocoaLumberjack/issues/883


Fix:
[https://github.com/DinosaurDad/CocoaLumberjack/commit/83499d8787e62e9da6f5d741f60c040d260b4fde](https://github.com/DinosaurDad/CocoaLumberjack/commit/83499d8787e62e9da6f5d741f60c040d260b4fde)

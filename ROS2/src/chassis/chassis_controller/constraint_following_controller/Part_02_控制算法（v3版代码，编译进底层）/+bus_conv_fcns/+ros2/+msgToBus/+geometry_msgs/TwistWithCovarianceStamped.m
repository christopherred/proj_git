function slBusOut = TwistWithCovarianceStamped(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    currentlength = length(slBusOut.header);
    for iter=1:currentlength
        slBusOut.header(iter) = bus_conv_fcns.ros2.msgToBus.std_msgs.Header(msgIn.header(iter),slBusOut(1).header(iter),varargin{:});
    end
    slBusOut.header = bus_conv_fcns.ros2.msgToBus.std_msgs.Header(msgIn.header,slBusOut(1).header,varargin{:});
    currentlength = length(slBusOut.twist);
    for iter=1:currentlength
        slBusOut.twist(iter) = bus_conv_fcns.ros2.msgToBus.geometry_msgs.TwistWithCovariance(msgIn.twist(iter),slBusOut(1).twist(iter),varargin{:});
    end
    slBusOut.twist = bus_conv_fcns.ros2.msgToBus.geometry_msgs.TwistWithCovariance(msgIn.twist,slBusOut(1).twist,varargin{:});
end

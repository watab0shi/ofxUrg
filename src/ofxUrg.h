#pragma once

#include "Urg_Driver.h"

#define OFX_URG_BEGIN_NAMESPACE namespace ofx { namespace Urg {
#define OFX_URG_END_NAMESPACE } }

OFX_URG_BEGIN_NAMESPACE

static const string DEFAULT_HOST = "192.168.0.10";
static const int DEFAULT_PORT = 10940;

enum ConnectionType
{
    SERIAL,
    ETHERNET
};
enum Mode
{
    DISTANCE,
    DISTANCE_INTENSITY,
    MULTIECHO,
    MULTIECHO_INTENSITY
};

typedef vector<long> Frame;

class Device : public ofThread
{
    
public:
    Device()
    :device_or_ip_name(DEFAULT_HOST)
    ,baudrate_or_port_number(DEFAULT_PORT)
    ,connect_type(ETHERNET)
    ,mode(DISTANCE)
    ,active(false)
    ,is_frame_new(false)
    {}
    
    virtual ~Device()
    {
        if (isThreadRunning()) ofThread::waitForThread();
        if (urg.is_open()) urg.close();
    }
    
    void setMode(const Mode& mode) { this->mode = mode; }
    bool setupSerial(const string& device_name = "", int baudrate = 115200);
    bool setupEthernet(const string& ip = DEFAULT_HOST, int port = DEFAULT_PORT);
    bool open();
    void close();
    bool start();
    void stop();
    void update();
    void drawDebug(float width = ofGetWindowWidth(), float height = ofGetWindowHeight()) const;
    void drawDebugPolar() const;
    bool isFrameNew() const { return is_frame_new; }
    
    string  productType(void)           const { return urg.product_type(); }
    string  firmwareVersion(void)       const { return urg.firmware_version(); }
    string  serialId(void)              const { return urg.serial_id(); }
    string  status(void)                const { return urg.status(); }
    string  state(void)                 const { return urg.state(); }
    
    int     minStep(void)               const { return urg.min_step(); }
    int     maxStep(void)               const { return urg.max_step(); }
    long    minDistance(void)           const { return urg.min_distance(); }
    long    maxDistance(void)           const { return urg.max_distance(); }
    long    scanUsec(void)              const { return urg.scan_usec(); }
    int     maxDataSize(void)           const { return urg.max_data_size(); }
    int     maxEchoSize(void)           const { return urg.max_echo_size(); }
    
    double  index2rad(int index)        const { return urg.index2rad(index); }
    double  index2deg(int index)        const { return urg.index2deg(index); }
    int     rad2index(double radian)    const { return urg.rad2index(radian); }
    int     deg2index(double degree)    const { return urg.deg2index(degree); }
    int     rad2step(double radian)     const { return urg.rad2step(radian); }
    int     deg2step(double degree)     const { return urg.deg2step(degree); }
    double  step2rad(int step)          const { return urg.step2rad(step); }
    double  step2deg(int step)          const { return urg.step2deg(step); }
    int     step2index(int step)        const { return urg.step2index(step); }
    
    const Frame& getData() const { return data; }
    long getData(int index) const { return data.at(index); }
    const vector<unsigned short>& getIntensity() const { return intensity; }
    unsigned short getIntensity(int index) const { return intensity.at(index); }
    
    double getFps() const { return fps.getFps(); }
protected:
    
    void threadedFunction();
    
    string device_or_ip_name;
    int baudrate_or_port_number;
    ConnectionType connect_type;
    Mode mode;
    qrk::Urg_driver urg;
    
    bool active;
    bool is_frame_new;
    vector<long> data, data_buffer;
    vector<unsigned short> intensity, intensity_buffer;
    long timestamp;
    ofFpsCounter fps;
};

// coord space is defined as
// Y-up and X-right (same as default oF space)
// it's more like naive interpretation of LIDAR orientation.
class Processor : public Device, public ofNode
{
public:
    struct Cluster
    {
        pair<int, ofVec3f> start_point;
        pair<int, ofVec3f> end_point;
        vector<ofVec3f> points;
        ofVec3f centroid;
        
        void draw();
    };
public:
    // parameters
    float valid_range_min;
    float valid_range_max;
    ofVec3f valid_aabb_min;
    ofVec3f valid_aabb_max;

    float scale; // by default, converting mm to cm
    float clustering_dist_th; // in cm..?

protected:
    // data
    vector<ofVec3f> points_3d;
    vector<Cluster> clusters;
    

public:
    Processor()
    :valid_range_min(0)
    ,valid_range_max(FLT_MAX)
    ,valid_aabb_min(ofVec3f(std::numeric_limits<float>::lowest()))
    ,valid_aabb_max(ofVec3f(std::numeric_limits<float>::max()))
    ,clustering_dist_th(20)
    ,scale(0.1)
    {}
    
    void update();
    
    const vector<Cluster>& getClusters() const { return clusters; }
    const vector<ofVec3f>& getPoints() const { return points_3d; }
    
    void drawDebug3d();
    void drawDebugPolarOfSpace();
};

OFX_URG_END_NAMESPACE

namespace ofxUrg = ofx::Urg;

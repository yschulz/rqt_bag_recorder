#ifndef RQT_BAG_RECORDER__BAG_RECORDER_UTILITIES_HPP
#define RQT_BAG_RECORDER__BAG_RECORDER_UTILITIES_HPP

#include <string>

template<typename T>
std::string toStringFileSize(T bytes){
    std::stringstream ss;

    if(bytes > (T) 1000000000000LL){
        ss << bytes / (T) 1000000000000LL << "." << bytes % (T) 1000000000000LL / (T) 10000000000LL  << " TB";
    }
    if(bytes > (T) 1000000000LL){
        ss << bytes / (T) 1000000000LL << "." << bytes % (T) 1000000000LL / (T) 10000000LL  << " GB";
    }
    else if(bytes > (T) 1000000LL){
        ss << bytes / (T) 1000000LL << "." << bytes %  (T) 1000000LL / (T) 10000LL << " MB";
    }
    else if(bytes > (T) 1000LL){
        ss << bytes / (T) 1000LL << "." << bytes % (T) 1000LL / (T) 10LL << " KB";
    }
    else{
        ss << bytes << " B";
    }
    return ss.str();
}

template<typename T>
std::string toStringSeconds(T seconds){
    std::stringstream ss;
    size_t i_seconds = seconds;


    auto h = i_seconds / (60 * 60);
    i_seconds -= h * (60 * 60);

    auto m = i_seconds / (60);
    i_seconds -= m * (60);

    auto s = i_seconds;

    if(h) ss << h << " h ";
    if(m) ss << m << " m ";
    if(s) ss << s << " s";
    return ss.str();
}

#endif
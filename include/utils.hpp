//
// Created by xinyang on 2020/10/14.
//

#ifndef STEREOSLAM_UTILS_HPP
#define STEREOSLAM_UTILS_HPP

#include <memory>

// utils
template<class T>
struct ExportProtectConstructor : public T {
    template<class ...Ts>
    explicit ExportProtectConstructor(Ts &&...args): T(std::forward<Ts>(args)...) {}
};

template<class T>
struct EnableBasicTypes : public std::enable_shared_from_this<T> {
    using sPtr = std::shared_ptr<T>;
    using wPtr = std::weak_ptr<T>;

    using sMap = std::unordered_map<size_t, sPtr>;
    using wMap = std::unordered_map<size_t, wPtr>;

    using sVec = std::vector<sPtr>;
    using wVec = std::vector<wPtr>;

    using sLst = std::list<sPtr>;
    using wLst = std::list<wPtr>;

    template<class ...Ts>
    static sPtr create(Ts &&...args) {
        static size_t factory_id = 0;
        auto ptr = std::make_shared<ExportProtectConstructor<T>>(std::forward<Ts>(args)...);
        ptr->_id = factory_id++;
        return ptr;
    }

    inline size_t id() { return _id; }

private:
    size_t _id{0};
};

#endif //STEREOSLAM_UTILS_HPP

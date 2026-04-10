#pragma once

#include <stdint.h>
#include <optional>
#include <variant>

// ComponentBase：控制组件基类，继承者必须实现 update()
class ComponentBase {
public:
    virtual void update(uint32_t timestamp) = 0;
};

template<typename T> class InputPort;

// OutputPort：输出端口，存储本周期计算结果供 InputPort 消费
// age_=0 表示本周期新值，每周期开始调用 reset() 使 age_++
template<typename T>
class OutputPort {
public:
    OutputPort(T val) : content_(val) {}
    void operator=(T value) { content_ = value; age_ = 0; } // 赋值时 age_ 归零
    void reset() { age_++; }                                // 周期开始时调用
    std::optional<T> present()  { return age_ == 0 ? std::make_optional(content_) : std::nullopt; } // 本周期值
    std::optional<T> previous() { return age_ == 1 ? std::make_optional(content_) : std::nullopt; } // 上周期值
    std::optional<T> any()      { return content_; } // 任意时刻值（线程安全，前提 T 读写原子）
private:
    uint32_t age_ = 2; // 2=未初始化，0=本周期，1=上周期
    T content_;
};

// InputPort：输入端口，来源可以是：OutputPort* / T* / 内部值 / nullptr(nullopt)
template<typename T>
class InputPort {
public:
    void connect_to(OutputPort<T>* p) { content_ = p; }  // 连接到 OutputPort（最常用）
    void connect_to(T* p)              { content_ = p; }  // 连接到外部变量
    void disconnect()                  { content_ = (OutputPort<T>*)nullptr; }

    std::optional<T> present() {
        if (content_.index() == 2) {
            auto* p = std::get<2>(content_);
            return p ? p->present() : std::nullopt;
        } else if (content_.index() == 1) {
            auto* p = std::get<1>(content_);
            return p ? std::make_optional(*p) : std::nullopt;
        }
        return std::get<0>(content_);
    }

    std::optional<T> any() {
        if (content_.index() == 2) {
            auto* p = std::get<2>(content_);
            return p ? p->any() : std::nullopt;
        } else if (content_.index() == 1) {
            auto* p = std::get<1>(content_);
            return p ? std::make_optional(*p) : std::nullopt;
        }
        return std::get<0>(content_);
    }

private:
    std::variant<T, T*, OutputPort<T>*> content_;
};

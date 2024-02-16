#ifndef ARTERY_DATAOBJECT_H_
#define ARTERY_DATAOBJECT_H_

#include <memory>
#include <cassert>

namespace artery
{

template<class T>
class DataObject
{
public:
    DataObject(const DataObject&) = default;
    DataObject& operator=(const DataObject&) = default;

    DataObject(T&& t) : m_t_wrapper(std::make_shared<const T>(std::move(t))) {}
    DataObject(const T& t) : m_t_wrapper(std::make_shared<const T>(t)) {}
    DataObject(const std::shared_ptr<const T>& ptr) : m_t_wrapper(ptr) { assert(m_t_wrapper); }

    DataObject& operator=(T&& t) { m_t_wrapper = std::make_shared<const T>(std::move(t)); return *this; }
    DataObject& operator=(const T& t) { m_t_wrapper = std::make_shared<const T>(t); return *this; }
    DataObject& operator=(const std::shared_ptr<const T>& ptr) { m_t_wrapper = ptr; assert(m_t_wrapper); return *this; }

    const T& value() const { assert(m_t_wrapper); return *m_t_wrapper; }
    std::shared_ptr<const T> shared_ptr() const { assert(m_t_wrapper); return m_t_wrapper; }

private:
    std::shared_ptr<const T> m_t_wrapper;
};

} // namespace artery

#endif /* ARTERY_DATAOBJECT_H_ */
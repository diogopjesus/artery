#include "artery/application/DataObject.h"
#include <cassert>

namespace artery
{

template<typename T>
DataObject<T>::DataObject(T&& t) :
    m_t_wrapper(std::make_shared<T>(std::move(t)))
{
}

template<typename T>
DataObject<T>& DataObject<T>::operator=(T&& t)
{
    m_t_wrapper = std::make_shared<T>(std::move(t));
    return *this;
}

template<typename T>
DataObject<T>::DataObject(const T& t) :
    m_t_wrapper(std::make_shared<T>(t))
{
}

template<typename T>
DataObject<T>& DataObject<T>::operator=(const T& t)
{
    m_t_wrapper = std::make_shared<T>(t);
    return *this;
}

template<typename T>
DataObject<T>::DataObject(const std::shared_ptr<const T>& ptr) :
    m_t_wrapper(ptr)
{
    assert(m_t_wrapper);
}

template<typename T>
DataObject<T>& DataObject<T>::operator=(const std::shared_ptr<const T>& ptr)
{
    m_t_wrapper = ptr;
    assert(m_t_wrapper);
    return *this;
}

template<typename T>
const T& DataObject<T>::value() const
{
    assert(m_t_wrapper);
    return *m_t_wrapper;
}

template<typename T>
std::shared_ptr<const T> DataObject<T>::shared_ptr() const
{
    assert(m_t_wrapper);
    return m_t_wrapper;
}

} // namespace artery
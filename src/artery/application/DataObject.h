#ifndef ARTERY_DATAOBJECT_H_
#define ARTERY_DATAOBJECT_H_

#include <memory>

namespace artery
{

template<typename T>
class DataObject
{
public:
    DataObject(const DataObject&) = default;
    DataObject& operator=(const DataObject&) = default;

    DataObject(T&& t);
    DataObject& operator=(T&& t);

    DataObject(const T&);
    DataObject& operator=(const T&);

    DataObject(const std::shared_ptr<const T>&);
    DataObject& operator=(const std::shared_ptr<const T>&);

    const T& value() const;
    std::shared_ptr<const T> shared_ptr() const;

private:
    std::shared_ptr<const T> m_t_wrapper;
};

} // namespace artery

#endif /* ARTERY_DATAOBJECT_H_ */
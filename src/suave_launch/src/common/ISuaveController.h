#ifndef ISUAVECONTROLLER_H
#define ISUAVECONTROLLER_H

class
ISuaveController
{
public:
    ISuaveController() = default;

    virtual ~ISuaveController() = default;

    virtual void start() = 0;
};

#endif //ISUAVECONTROLLER_H

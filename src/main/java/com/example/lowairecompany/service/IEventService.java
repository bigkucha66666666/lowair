package com.example.lowairecompany.service;

import com.example.lowairecompany.pojo.Event;
import com.example.lowairecompany.pojo.User;
import java.util.List;

public interface IEventService {
    void add(Event event);
    //查询事件
    Event get(Integer event_Id);

    Event edit(Event event);

    Event delete(Integer event_Id);
    
    //获取所有事件名称
    List<String> getAllEventNames();
}

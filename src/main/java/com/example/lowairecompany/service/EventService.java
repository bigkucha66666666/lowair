package com.example.lowairecompany.service;

import com.example.lowairecompany.dao.EventDao;
import com.example.lowairecompany.pojo.Event;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

@Service
public class EventService implements IEventService {
    @Autowired
    EventDao eventdao;
    @Override
    public void add(Event event){
        eventdao.save(event);
    }
    @Override
    public Event get(Integer id){
        return eventdao.findById(id).orElseThrow(() -> new IllegalArgumentException("Event 不存在"));
    }
    @Override
    public Event edit(Event event){
        return eventdao.save(event);
    }
    @Override
    public Event delete(Integer id){
        Event existing=eventdao.findById(id).orElseThrow(() -> new IllegalArgumentException("Event 不存在"));
        eventdao.deleteById(id);
        return existing;
    }
    
    @Override
    public List<String> getAllEventNames() {
        Iterable<Event> events = eventdao.findAll();
        return StreamSupport.stream(events.spliterator(), false)
                .map(Event::getEvent_name)
                .collect(Collectors.toList());
    }
}

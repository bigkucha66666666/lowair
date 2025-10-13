package com.example.lowairecompany.controller;

import com.example.lowairecompany.pojo.ResponseMessage;
import com.example.lowairecompany.pojo.Event;
import com.example.lowairecompany.service.IEventService;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.*;

@RestController    //接口对象返回对象 转换成json文本
@RequestMapping("/event")    //前端访问接口
public class EventController {
    @Autowired
    IEventService eventService;
    
    //增加
    @PostMapping
    public ResponseMessage add(@RequestBody Event event){
        eventService.add(event);
        return ResponseMessage.success(null);
    }
    
    @GetMapping("/{eventId}")
    public ResponseMessage get(@PathVariable Integer eventId){
        Event eventNew = eventService.get(eventId);
        return ResponseMessage.success(eventNew);
    }
    
    //修改
    @PutMapping
    public ResponseMessage edit(@Validated @RequestBody Event event){
        Event eventNew = eventService.edit(event);
        return ResponseMessage.success(eventNew);
    }
    
    @DeleteMapping("/{eventId}")
    public ResponseMessage delete(@PathVariable Integer eventId) {
        Event eventNew = eventService.delete(eventId);
        return ResponseMessage.success(eventNew);
    }
    
    //获取所有事件名称
    @GetMapping("/names")
    public ResponseMessage getAllEventNames() {
        return ResponseMessage.success(eventService.getAllEventNames());
    }
}

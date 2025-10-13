package com.example.lowairecompany.controller;

import com.example.lowairecompany.pojo.Airfiled;
import com.example.lowairecompany.pojo.ResponseMessage;
import com.example.lowairecompany.service.IAirfiledService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.*;
import java.util.List;

@RestController
@RequestMapping("/airfiled")
public class AirfiledController {

    @Autowired
    private IAirfiledService airfiledService;

    @PostMapping
    public ResponseMessage<Void> add(@RequestBody Airfiled airfiled) {
        airfiledService.add(airfiled);
        return ResponseMessage.success(null);
    }

    @GetMapping("/{id}")
    public ResponseMessage<Airfiled> get(@PathVariable Integer id) {
        return ResponseMessage.success(airfiledService.get(id));
    }

    @PutMapping
    public ResponseMessage<Airfiled> edit(@Validated @RequestBody Airfiled airfiled) {
        return ResponseMessage.success(airfiledService.edit(airfiled));
    }

    @DeleteMapping("/{id}")
    public ResponseMessage<Airfiled> delete(@PathVariable Integer id) {
        return ResponseMessage.success(airfiledService.delete(id));
    }
    
    //获取所有机场名称
    @GetMapping("/names")
    public ResponseMessage getAllAirfiledNames() {
        return ResponseMessage.success(airfiledService.getAllAirfiledNames());
    }
    
    //获取所有机场数据
    @GetMapping("/all")
    public ResponseMessage getAllAirfileds() {
        return ResponseMessage.success(airfiledService.getAllAirfileds());
    }
    
    //批量删除机场
    @DeleteMapping("/batch")
    public ResponseMessage deleteBatch(@RequestBody List<Integer> ids) {
        airfiledService.deleteBatch(ids);
        return ResponseMessage.success(null);
    }
}
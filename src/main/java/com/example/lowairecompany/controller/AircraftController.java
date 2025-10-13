package com.example.lowairecompany.controller;
import com.example.lowairecompany.pojo.Aircraft;
import com.example.lowairecompany.pojo.ResponseMessage;
import com.example.lowairecompany.service.IAircraftService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.*;
import java.util.List;

@RestController
@RequestMapping("/aircraft")
public class AircraftController {

    @Autowired
    private IAircraftService aircraftService;

    @PostMapping
    public ResponseMessage<Void> add(@RequestBody Aircraft aircraft) {
        aircraftService.add(aircraft);
        return ResponseMessage.success(null);
    }

    @GetMapping("/{id}")
    public ResponseMessage<Aircraft> get(@PathVariable Integer id) {
        return ResponseMessage.success(aircraftService.get(id));
    }

    @PutMapping
    public ResponseMessage<Aircraft> edit(@Validated @RequestBody Aircraft aircraft) {
        return ResponseMessage.success(aircraftService.edit(aircraft));
    }

    @DeleteMapping("/{id}")
    public ResponseMessage<Aircraft> delete(@PathVariable Integer id) {
        return ResponseMessage.success(aircraftService.delete(id));
    }
    
    //获取所有飞行器类型
    @GetMapping("/types")
    public ResponseMessage getAllAircraftTypes() {
        return ResponseMessage.success(aircraftService.getAllAircraftTypes());
    }
    
    //获取所有飞行器数据
    @GetMapping("/all")
    public ResponseMessage getAllAircrafts() {
        return ResponseMessage.success(aircraftService.getAllAircrafts());
    }
    
    //批量删除飞行器
    @DeleteMapping("/batch")
    public ResponseMessage deleteBatch(@RequestBody List<Integer> ids) {
        aircraftService.deleteBatch(ids);
        return ResponseMessage.success(null);
    }
}
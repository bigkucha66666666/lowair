package com.example.lowairecompany.controller;
import com.example.lowairecompany.pojo.AirZone;
import com.example.lowairecompany.pojo.ResponseMessage;
import com.example.lowairecompany.service.IAirZoneService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.*;
import java.util.List;

@RestController
@RequestMapping("/airzone")
public class AirZoneController {

    @Autowired
    private IAirZoneService airZoneService;

    @PostMapping
    public ResponseMessage<Void> add(@RequestBody AirZone airZone) {
        airZoneService.add(airZone);
        return ResponseMessage.success(null);
    }

    @GetMapping("/{zoneId}")
    public ResponseMessage<AirZone> get(@PathVariable Integer zoneId) {
        return ResponseMessage.success(airZoneService.get(zoneId));
    }

    @PutMapping
    public ResponseMessage<AirZone> edit(@Validated @RequestBody AirZone airZone) {
        return ResponseMessage.success(airZoneService.edit(airZone));
    }

    @DeleteMapping("/{zoneId}")
    public ResponseMessage<AirZone> delete(@PathVariable Integer zoneId) {
        return ResponseMessage.success(airZoneService.delete(zoneId));
    }
    
    //获取所有空域名称
    @GetMapping("/names")
    public ResponseMessage getAllZoneNames() {
        return ResponseMessage.success(airZoneService.getAllZoneNames());
    }
    
    //获取所有空域数据
    @GetMapping("/all")
    public ResponseMessage getAllZones() {
        return ResponseMessage.success(airZoneService.getAllZones());
    }
    
    //批量删除空域
    @DeleteMapping("/batch")
    public ResponseMessage deleteBatch(@RequestBody List<Integer> zoneIds) {
        airZoneService.deleteBatch(zoneIds);
        return ResponseMessage.success(null);
    }
}

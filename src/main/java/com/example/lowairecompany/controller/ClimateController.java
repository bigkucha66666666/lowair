package com.example.lowairecompany.controller;

import com.example.lowairecompany.pojo.Climate;
import com.example.lowairecompany.pojo.ResponseMessage;
import com.example.lowairecompany.service.IClimateService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.*;
import java.util.List;

@RestController
@RequestMapping("/climate")
public class ClimateController {

    @Autowired
    private IClimateService climateService;

    @PostMapping
    public ResponseMessage<Void> add(@RequestBody Climate climate) {
        climateService.add(climate);
        return ResponseMessage.success(null);
    }

    @GetMapping("/{id}")
    public ResponseMessage<Climate> get(@PathVariable Integer id) {
        return ResponseMessage.success(climateService.get(id));
    }

    @PutMapping
    public ResponseMessage<Climate> edit(@Validated @RequestBody Climate climate) {
        return ResponseMessage.success(climateService.edit(climate));
    }

    @DeleteMapping("/{id}")
    public ResponseMessage<Climate> delete(@PathVariable Integer id) {
        return ResponseMessage.success(climateService.delete(id));
    }
    
    //获取所有天气数据
    @GetMapping("/all")
    public ResponseMessage getAllClimates() {
        return ResponseMessage.success(climateService.getAllClimates());
    }
    
    //批量删除天气数据
    @DeleteMapping("/batch")
    public ResponseMessage deleteBatch(@RequestBody List<Integer> ids) {
        climateService.deleteBatch(ids);
        return ResponseMessage.success(null);
    }
}

package com.example.lowairecompany.controller;

import com.example.lowairecompany.pojo.ResponseMessage;
import com.example.lowairecompany.pojo.Scene;
import com.example.lowairecompany.service.ISceneService;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.*;

@RestController    //接口对象返回对象 转换成json文本
@RequestMapping("/scene")    //前端访问接口
public class SceneController {
    @Autowired
    ISceneService sceneService;
    
    //增加
    @PostMapping
    public ResponseMessage add(@RequestBody Scene scene) {
        sceneService.add(scene);
        return ResponseMessage.success(null);
    }
    
    @GetMapping("/{sceneId}")
    public ResponseMessage get(@PathVariable Integer sceneId) {
        Scene sceneNew = sceneService.getScene(sceneId);
        return ResponseMessage.success(sceneNew);
    }
    
    //修改
    @PutMapping
    public ResponseMessage edit(@Validated @RequestBody Scene scene) {
        Scene sceneNew = sceneService.edit(scene);
        return ResponseMessage.success(sceneNew);
    }
    
    @DeleteMapping("/{sceneId}")
    public ResponseMessage delete(@PathVariable Integer sceneId) {
        Scene sceneNew = sceneService.delete(sceneId);
        return ResponseMessage.success(sceneNew);
    }
}
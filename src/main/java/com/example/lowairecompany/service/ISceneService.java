package com.example.lowairecompany.service;

import com.example.lowairecompany.pojo.Scene;

public interface ISceneService {
    //增加场景
    void add(Scene scene);
    //查询场景
    Scene getScene(Integer sceneId);
    //修改场景
    Scene edit(Scene scene);
    //删除场景
    Scene delete(Integer sceneId);
}
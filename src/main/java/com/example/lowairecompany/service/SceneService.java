package com.example.lowairecompany.service;

import com.example.lowairecompany.dao.SceneDao;
import com.example.lowairecompany.pojo.Scene;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service  //标记成spring的bean
public class SceneService implements ISceneService {
    @Autowired
    SceneDao sceneDao;
    
    @Override
    public void add(Scene scene) {
        sceneDao.save(scene);
    }
    
    @Override
    public Scene getScene(Integer sceneId) {
        return sceneDao.findById(sceneId).orElseThrow(() -> new IllegalArgumentException("场景不存在"));
    }
    
    @Override
    public Scene edit(Scene scene) {
        return sceneDao.save(scene);
    }
    
    @Override
    public Scene delete(Integer sceneId) {
        Scene existing = sceneDao.findById(sceneId).orElseThrow(() -> new IllegalArgumentException("场景不存在"));
        sceneDao.deleteById(sceneId);
        return existing;
    }
}
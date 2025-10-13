package com.example.lowairecompany.controller;

import com.example.lowairecompany.pojo.Need;
import com.example.lowairecompany.pojo.ResponseMessage;
import com.example.lowairecompany.service.INeedService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.validation.annotation.Validated;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/need")
public class NeedController {

    @Autowired
    private INeedService needService;

    @PostMapping
    public ResponseMessage<Void> add(@RequestBody Need need) {
        needService.add(need);
        return ResponseMessage.success(null);
    }

    @GetMapping("/{id}")
    public ResponseMessage<Need> get(@PathVariable Integer id) {
        return ResponseMessage.success(needService.get(id));
    }

    @PutMapping
    public ResponseMessage<Need> edit(@Validated @RequestBody Need need) {
        return ResponseMessage.success(needService.edit(need));
    }

    @DeleteMapping("/{id}")
    public ResponseMessage<Need> delete(@PathVariable Integer id) {
        return ResponseMessage.success(needService.delete(id));
    }
}



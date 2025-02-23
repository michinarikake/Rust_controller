use std::fs::{self, File};
use std::io::{Write, BufWriter};
// use std::path::Path;
use chrono::Local;
use crate::repositry::loggable_trait::Loggable;

/// **汎用的な `Logger`**
pub struct Logger {
    file: BufWriter<File>,
    log_entries: Vec<Box<dyn Loggable>>,
    headers_written: bool,
}

impl Logger {
    /// **日付ごとのログディレクトリを作成し、適切なパスを生成**
    fn generate_log_path(filename: &str) -> String {
        let date = Local::now().format("%Y-%m-%d").to_string();
        let dir_path = format!("data/{}/", date);
        let file_path = format!("{}{}_{}", dir_path, date, filename);

        // ディレクトリを作成
        if let Err(e) = fs::create_dir_all(&dir_path) {
            eprintln!("Failed to create log directory: {}", e);
        }

        file_path
    }

    /// **新しいログファイルを作成**
    pub fn new(filename: &str) -> std::io::Result<Self> {
        let file_path = Self::generate_log_path(filename);
        let file = File::create(file_path)?;
        let writer = BufWriter::new(file);

        Ok(Self {
            file: writer,
            log_entries: Vec::new(),
            headers_written: false,
        })
    }

    /// **ログにデータを追加**
    pub fn add_entry<T: Loggable + 'static>(&mut self, data: T) {
        self.log_entries.push(Box::new(data));
    }

    /// **バッファにあるデータを一括で出力**
    pub fn log(&mut self, step: usize) {
        if !self.headers_written {
            let headers: Vec<String> = self.log_entries.iter().map(|entry| entry.header()).collect();
            let header_line = format!("Step,{}\n", headers.join(","));
            if let Err(e) = self.file.write_all(header_line.as_bytes()) {
                eprintln!("Failed to write header: {}", e);
            }
            self.headers_written = true;
        }

        let log_data: Vec<String> = self.log_entries.iter().map(|entry| entry.output_log()).collect();
        let log_entry = format!("{},{}\n", step, log_data.join(","));

        if let Err(e) = self.file.write_all(log_entry.as_bytes()) {
            eprintln!("Failed to write log entry: {}", e);
        }

        // ログエントリをクリア（次のステップのため）
        self.log_entries.clear();
    }

    /// **ファイルをフラッシュ**
    pub fn flush(&mut self) {
        if let Err(e) = self.file.flush() {
            eprintln!("Failed to flush log file: {}", e);
        }
    }
}

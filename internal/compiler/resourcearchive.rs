// Copyright © SixtyFPS GmbH <info@slint-ui.com>
// SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Slint-commercial

use byteorder::{LittleEndian, WriteBytesExt};
use std::io::{Read, Write};
use std::path::Path;

use crate::embedded_resources::{EmbeddedResourcesKind, PixelFormat};
use crate::object_tree::Document;

pub fn archive_resources(output_directory: &Path, doc: &Document) -> std::io::Result<()> {
    let file = tempfile::NamedTempFile::new_in(&output_directory)?;
    let mut zip = zip::ZipWriter::new(file);

    let resources = doc.root_component.embedded_file_resources.borrow();
    for (resource_path, resource) in resources.iter() {
        match &resource.kind {
            EmbeddedResourcesKind::RawData => {
                let file_name = format!("{}.raw", resource.id);
                zip.start_file(file_name, zip::write::FileOptions::default())?;
                let file =
                    crate::fileaccess::load_file(std::path::Path::new(resource_path)).unwrap(); // embedding pass ensured that the file exists
                match file.builtin_contents {
                    Some(static_data) => {
                        zip.write(static_data)?;
                    }
                    None => {
                        let mut f = std::fs::File::open(Path::new(file.path.as_ref()))?;
                        let mut data = Vec::with_capacity(f.metadata()?.len() as usize);
                        f.read_to_end(&mut data)?;
                        zip.write(&data)?;
                    }
                }
            }
            EmbeddedResourcesKind::TextureData(crate::embedded_resources::Texture {
                data,
                format,
                rect,
                total_size: crate::embedded_resources::Size { width, height },
                original_size:
                    crate::embedded_resources::Size { width: unscaled_width, height: unscaled_height },
            }) => {
                let file_name = format!("{}.tex", resource.id);
                zip.start_file(file_name, zip::write::FileOptions::default())?;
                zip.write_u32::<LittleEndian>(*width)?;
                zip.write_u32::<LittleEndian>(*height)?;
                zip.write_u32::<LittleEndian>(*unscaled_width)?;
                zip.write_u32::<LittleEndian>(*unscaled_height)?;
                zip.write_i32::<LittleEndian>(rect.x())?;
                zip.write_i32::<LittleEndian>(rect.y())?;
                zip.write_u32::<LittleEndian>(rect.width())?;
                zip.write_u32::<LittleEndian>(rect.height())?;
                match format {
                    PixelFormat::Rgb => zip.write_u8(0)?,
                    PixelFormat::Rgba => zip.write_u8(1)?,
                    PixelFormat::AlphaMap(_) => todo!(),
                }
                zip.write_u32::<LittleEndian>(data.len() as u32)?;
                for byte in data {
                    zip.write_u8(*byte)?;
                }
            }
            EmbeddedResourcesKind::BitmapFontData(_) => {
                // not embedded
            }
        }
    }

    let file = zip.finish()?;

    file.persist(output_directory.join("assset.zip"))?;
    Ok(())
}
